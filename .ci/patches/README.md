# BlueZ patch audit

The package builder targets BlueZ 5.87 and applies only the patches listed in
this directory. The audit rule is strict: a daemon patch is retained only when
the required state or operation is below every public D-Bus boundary available
to the ROS 2 package. If a reliable package-side D-Bus implementation exists,
the daemon patch must be removed.

The former `.ci/pkg_bluez` overlay was removed. The package no longer copies
configuration files over the distribution's BlueZ defaults. The former
`bluez-explicit-disconnect.patch` was also removed: the ROS service can perform
the required handoff with standard `Trusted=false`, `Device1.Disconnect`, and
`Trusted=true` D-Bus calls while preserving the bond. The former
`bluez-advertising-data-update.patch` was removed after the ROS service's
standard `LEAdvertisingManager1` unregister/register path passed a hardware
A/B at the unchanged payload and source rate. The earlier
`bluez-mesh-single-tx-worker.patch` was superseded by the audited radio
scheduler patch and is not applied separately.

## Control boundary

GATT and connectionless advertisements are controlled through the standard
`org.bluez` D-Bus APIs. Mesh lifecycle, provisioning, key management, model
configuration, transmission and cancellation are controlled through
`org.bluez.mesh` D-Bus APIs. The application exports `Application1`,
`Element1`, `ProvisionAgent1`, `Provisioner1`, `Attention1`, and
`ObjectManager`; it calls `Network1`, `Node1`, and `Management1`.
Starting and stopping `bluetooth-meshd` is also done through D-Bus. No
`mesh-cfgclient`, raw HCI command, or direct daemon-state manipulation is used
at runtime.

Three methods added by `bluez-mesh-sar-queue.patch` remain D-Bus methods:
`Node1.SendUnqueued`, `Node1.SendPending`, and `Node1.CancelSend`. They
expose ownership which the standard fire-and-forget `Node1.Send` omits.

## Necessity matrix

| Patch | Defect and evidence | Why the ROS package cannot implement it |
| --- | --- | --- |
| `bluez-adapter-mode-completion.patch` | A successful or idempotent management completion may arrive without a later `NEW_SETTINGS` change, leaving BlueZ's internal `pending_settings` bit set and all later mode changes returning Busy. Both internal and D-Bus property completion callbacks now release the exact pending bit on every completion path. | D-Bus exposes the resulting adapter property, not `pending_settings` or the management request completion. Retrying cannot clear a permanently stale internal bit. Restarting `bluetoothd` from ROS would disrupt every client and is recovery, not a correct implementation. |
| `bluez-mesh-radio-scheduler.patch` | BlueZ 5.87's generic HCI backend can overlap advertising workers and scan-programming chains, overwrite current TX ownership, ignore command failures, and parse only the first advertising report without complete bounds checks. Upstream [#206](https://github.com/bluez/bluez/issues/206), [#561](https://github.com/bluez/bluez/issues/561), [#1950](https://github.com/bluez/bluez/issues/1950), [#2337](https://github.com/bluez/bluez/issues/2337), and [#2353](https://github.com/bluez/bluez/issues/2353) independently document persistent or silent generic-backend TX failures, scan/advertise conflicts, or D-Bus success without RF transmission. Hardware traces also showed uav99 rejecting 20 ms advertising and accepting 100 ms. The patch serializes HCI transactions, owns deferred callbacks, validates completions and report lengths, and derives packet dwell from the accepted interval. | `bluetooth-meshd` exclusively owns the raw-HCI bearer. D-Bus exposes neither scan/advertising command sequencing nor controller completion callbacks, active bearer packets, deferred callbacks, or batched report bytes. A ROS retry sees only that `Node1.Send` was accepted, exactly the false-success condition in the upstream reports. |
| `bluez-mesh-sar-queue.patch` | Upstream lower transport queues segmented work without application ownership. Its ACK lookup is under-specified by SeqZero, partial ACK progress can leave stale repetitions, and queued encrypted packets can outlive an overlay. The patch matches source/destination/subnet/SeqZero/OBO, accumulates partial bitmaps, refreshes only missing segments, coalesces ACKs, and provides exact attachment-scoped handles. | Segment ACK bitmaps, encrypted network PDUs, SAR timers, Friend/OBO state, replay state and bearer queues exist only inside `bluetooth-meshd`. Standard `Node1.Send` has no handle, pending query, cancellation, or pre-encryption Busy result. Application receipts alone cannot cancel stale encrypted packets or safely decide whether native repair is still active. |
| `bluez-mesh-local-pb-adv.patch` | The Mesh API defines omitted `AddNode` `Server` as local provisioning, but BlueZ 5.87 substitutes the local node's primary address and routes the request through its Remote Provisioning Server. On the two radios that self-routed link repeatedly opened and closed without completing; direct PB-ADV completed enrollment. | The caller already makes the only available D-Bus choice—omit `Server`. D-Bus exposes no “force local PB-ADV” flag and the ROS package must not open the raw provisioning bearer beside the daemon. |
| `bluez-mesh-joined-provisioner-keyring.patch` | A node provisioned over PB-ADV receives operational NetKey/AppKey material in its node state, but BlueZ 5.87 does not populate that node's `Management1` keyring. `ExportKeys` then fails and the joined UAV cannot configure its local vendor model or provision the next peer. The patch copies only missing local persistent keys and also records AppKey Add. | The joined ROS process is not given plaintext keys by `JoinComplete` or Config Server messages. `ImportSubnet` requires the key it is supposed to recover, while `ExportKeys` is the failing operation. Inventing a new key would split the Mesh; sending keys over Zenoh would violate the Mesh trust and isolation boundary. |
| `bluez-mesh-reconcile-local-devkey.patch` | A stale self DevKey entry makes local Config Server traffic decrypt as remote-device-key traffic after identity reuse. The patch compares the self entry with the node's authoritative stored DevKey on attach and replaces only that entry. | `Management1.ImportRemoteNode` rejects a range overlapping the attached local node. D-Bus does not expose an operation to rewrite the daemon's self entry, and deleting/reimporting the local node would destroy the persistent identity the operation is meant to repair. |
| `bluez-mesh-userspace-aes-ccm.patch` | On uav99 the pristine ELL AES-CCM known-answer test fails because host security policy disables `algif_aead`; the same vector passes through OpenSSL EVP. The patch keeps BlueZ's startup known-answer check and replaces only the AES-CCM provider. | Network and application encryption/decryption occur inside `bluetooth-meshd` before and after D-Bus access messages. ROS cannot supply ciphertext without reimplementing/bypassing the Mesh stack, and it must not weaken the host's kernel-module security policy. |
| `bluez-mesh-sequence-reservation.patch` | BlueZ may transmit after an in-memory sequence increment but before that exact value is durable. A daemon or power restart can therefore reuse an already observed sequence number and be rejected by peer replay protection. The patch reserves a monotonic block synchronously before the attached node may transmit and never lowers the durable high-water mark at clean shutdown. | The network sequence is assigned to Config, control, relay and application traffic inside the daemon. D-Bus does not expose sequence allocation or a durable pre-send transaction, so ROS cannot reserve safely for traffic it does not originate. |

The upstream audit was refreshed at `5.87-290-g73e934b` (2026-09-29). No commit
in `5.87..73e934b` changes any patched Mesh source file. Current master still
clears `pending_settings` only on adapter-command failure, not on successful
no-change completion. All eight patches apply cleanly, in package order, to
both pristine 5.87 and that master revision; none is integrated upstream.
Package builds remain pinned to 5.87, and patch application is deliberately
fail-closed with `git apply --check`.

## ROS-side reliability design

The package does not use a fixed two-peer arbiter. For each channel it builds a
sorted unique ring from the local address and all fresh authenticated same-swarm
members. The current sender selects its sorted successor and marks only that
destination's unicast copy as the handoff; receivers never recompute the choice
from potentially lagging local membership views. Exactly the lowest current
live address initializes or recovers a lost turn. Configured but unheard probe
addresses may receive a bounded discovery transmission but never enter the ring.

A token holder fans the newest 60-byte topic value out as acknowledged unicast
to every live member. Every destination has independent native SAR ownership and
an application receipt. This avoids the poor group-primary behavior observed on
the target controllers and generalizes to N peers. A gitignored deterministic
five-peer model test covers shuffled/duplicate membership, every successor,
membership removal, sole recovery ownership, simultaneous initialization, and
offline-probe exclusion.

Acknowledged unicast now uses zero blanket source network retransmissions in
the example overlay. Native lower-transport SAR selectively retransmits missing
segments, while relay retransmission remains enabled for multi-hop forwarding.
The previous count of two transmitted every source PDU three times and delayed
turn handoff. The configured 10 Hz source rate and wire layouts are unchanged;
the service continues to retain only the newest value while the bearer is busy.

Each channel holds at most one blocking transaction and its newest waiting
value. Compact receipt queries let a receiver repeat its application receipt
after native repair. Receipts are scoped by source, AppKey, swarm, vendor opcode
and application sequence. Transfers are cancelled after receipt, expiry,
overlay replacement or application disconnect. Handles include the attachment
generation, preventing delayed cleanup from cancelling a replacement network.

Startup buffers the newest topic value until attach, AppKey binding and local
radio configuration are confirmed by Config Server status messages. No
fixed-delay assumption marks Mesh ready.

## Hardware evidence

Both the BlueZ daemons and ROS package compile on uav42 (x86-64) and uav99
(ARM64). Tests run with `rmw_zenoh_cpp` and a local probe on each UAV, so ROS
transport cannot carry the measured peer payload over Wi-Fi. The gitignored
probe derives every expected field from the sender and message timestamp,
validates exact timestamp, pose and twist values, counts duplicates, and records
end-to-end age and inter-delivery gaps.

Final 300-second bidirectional soaks passed in every mode with no corrupt or
duplicate ROS delivery. GATT delivered 2,537 exact samples from uav99 to uav42
and 2,494 in reverse (8.453/8.312 Hz), with 0.375/0.534 s maximum gaps and
0.434/0.220 s maximum ages. Advertisement delivered 906 and 2,324 exact
samples (3.019/7.746 Hz), with 1.798/0.521 s maximum gaps and 0.136/0.176 s
maximum ages. Mesh delivered 187 exact full-size samples in each direction
(0.623 Hz), with 4.219 s maximum gaps and 2.981/2.429 s maximum ages. Every
overlay reverted to default successfully.

Randomized process-restart campaigns exercised each mode with uav42 alone,
uav99 alone, and both UAVs restarting at independent times. Every campaign
recovered automatically through the D-Bus configuration service with exact
payloads and zero duplicate ROS delivery. One-sided GATT gaps stayed at or
below 2.199 s and Advertisement gaps at or below 4.000 s. One-sided Mesh gaps
were 5.980 s when uav42 restarted and 13.215/11.969 s when uav99, the current
turn holder, restarted. The overlapping two-sided campaign measured GATT gaps
of 2.065/1.967 s, Advertisement gaps of 3.394/1.216 s, and Mesh gaps of
17.039/15.479 s. The longer Mesh case is the bounded 12 s lost-turn safety
guard plus D-Bus reattachment while the deterministic recovery member also
restarts; delivery resumed without daemon restart or operator action.

A separate standard-use campaign kept uav99 active while uav42 returned to its
default configuration for eight seconds and then reapplied the same overlay.
The post-reapply windows delivered 360/372 exact GATT samples, 159/287 exact
Advertisement samples, and 23/26 exact Mesh samples, with no corruption or
duplicates. This also covered a peer remaining early while the other joined
late. All three directed mode changes and all three reapplications succeeded.

The immediately preceding configuration, with three transmissions for every
source network PDU and a 3 s lost-token deadline, failed a 60-second gate:
uav99→uav42 reached a 7.489 s gap while the lowest address reclaimed during a
healthy successor's SAR/ACK drain. The reverse direction reached 2.968 s. This
A/B result motivated zero blanket network retransmissions and a 12 s
lost-token-only recovery deadline.

A group-primary A/B run was rejected: after 40–45 seconds it had delivered only
two samples per direction, with roughly 17–21 s gaps and 18–20 s ages. Group
delivery is therefore not used as the primary N-peer data path on these radios.

Earlier patched-stack validation measured 26-byte advertisement messages at
4.60–4.96 Hz with gaps up to 1.30 s, and 60-byte GATT messages at
7.35–8.60 Hz with gaps up to 0.30 s. Six directed transitions between the three
modes, pending-connection handoff, one-sided overlay reset, adapter property
checks, and local ASan/UBSan probes passed. Those results are retained as
regression evidence but do not replace the current long/restart campaign.

The removed advertising-update patch has no remaining A/B gate: repeated
standard `LEAdvertisingManager1` unregister/register operation, the 300-second
soak, one-sided restarts, two-sided restarts, and one-sided overlay reapply all
passed on the unmodified BlueZ advertising D-Bus path. No package behavior
depends on that removed daemon change.

Only two physical UAV radios were available. N>2 membership, successor choice,
single recovery ownership, asymmetric membership views, removal, and offline
probe exclusion therefore have deterministic five-peer coverage rather than
an N>2 RF claim. Multi-hop RF performance remains environment-dependent and
must be characterized when a third radio is available; it is not evidence for
retaining any additional BlueZ patch.

## Primary upstream references

- [BlueZ Mesh D-Bus API](https://github.com/bluez/bluez/blob/master/doc/mesh-api.txt)
- [BlueZ advertising D-Bus API](https://github.com/bluez/bluez/blob/5.87/doc/org.bluez.LEAdvertisement.rst)
- [BlueZ management protocol](https://github.com/bluez/bluez/blob/5.87/doc/mgmt-protocol.rst)
- [BlueZ 5.87 generic Mesh backend](https://github.com/bluez/bluez/blob/5.87/mesh/mesh-io-generic.c)
- [Persistent Mesh HCI TX stall #206](https://github.com/bluez/bluez/issues/206)
- [Kernel 6.2 Mesh send failure #561](https://github.com/bluez/bluez/issues/561)
- [Continuous-scan send conflict #1950](https://github.com/bluez/bluez/issues/1950)
- [Persistent Mesh TX stall with live RX/D-Bus #2353](https://github.com/bluez/bluez/issues/2353)
- [Random-address send failure #2337](https://github.com/bluez/bluez/issues/2337)
- [Advertising/scanning address conflict #761](https://github.com/bluez/bluez/issues/761)
- [Bluetooth Mesh protocol: segmentation and acknowledgements](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/MshPRT_v1.1/out/en/index-en.html)

Every retained patch must continue to pass its targeted unit/sanitizer probe,
clean 5.87 application, both-architecture compilation, and relevant hardware
A/B. A BlueZ update reopens every necessity decision.
