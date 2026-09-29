# BlueZ patch audit

The package builder targets BlueZ 5.87 and applies the audited patches listed in
this directory. Each retained daemon patch owns state below the public D-Bus
boundary available to the ROS 2 package. Reliable D-Bus operations belong in
the ROS package, and implementing an operation there retires its corresponding
daemon patch.

The retired `.ci/pkg_bluez` overlay has left the distribution's BlueZ
configuration under package-manager ownership. Standard `Trusted=false`,
`Device1.Disconnect`, and `Trusted=true` D-Bus calls now perform the connection
handoff while preserving the bond, which retired
`bluez-explicit-disconnect.patch`. Hardware A/B results at the original payload
and source rate established standard `LEAdvertisingManager1`
unregister/register as the advertisement update path and retired
`bluez-advertising-data-update.patch`. The audited radio scheduler incorporates
the earlier single-worker change, so `bluez-mesh-single-tx-worker.patch` is
retired as a standalone patch.

## Control boundary

Standard `org.bluez` D-Bus APIs control GATT and connectionless advertisements.
The `org.bluez.mesh` D-Bus APIs control Mesh lifecycle, provisioning, key
management, model configuration, transmission, and cancellation. The
application exports `Application1`, `Element1`, `ProvisionAgent1`,
`Provisioner1`, `Attention1`, and `ObjectManager`; it calls `Network1`, `Node1`,
and `Management1`. Systemd's D-Bus manager starts and stops
`bluetooth-meshd`. Runtime control stays at these D-Bus boundaries; raw HCI and
daemon-private state remain owned by BlueZ. `mesh-cfgclient` is reserved for
manual diagnostics.

Three methods added by `bluez-mesh-sar-queue.patch` extend the D-Bus boundary:
`Node1.SendUnqueued`, `Node1.SendPending`, and `Node1.CancelSend`. They expose
explicit transfer ownership around the fire-and-forget `Node1.Send` operation.

## Necessity matrix

| Patch | Defect and evidence | Daemon ownership boundary |
| --- | --- | --- |
| `bluez-adapter-mode-completion.patch` | A successful or idempotent management completion may precede any `NEW_SETTINGS` signal, leaving BlueZ's internal `pending_settings` bit set and later mode changes returning Busy. Both internal and D-Bus property completion callbacks now release the exact pending bit on every completion path. | D-Bus returns the resulting adapter property; `pending_settings` and management completions remain private to BlueZ. Correctness therefore belongs in the daemon completion handler. A daemon restart is disruptive recovery for every connected client. |
| `bluez-mesh-radio-scheduler.patch` | BlueZ 5.87's generic HCI backend can overlap advertising workers and scan-programming chains, overwrite current TX ownership, drop command failures, and stop after the first advertising report in a batch. Upstream [#206](https://github.com/bluez/bluez/issues/206), [#561](https://github.com/bluez/bluez/issues/561), [#1950](https://github.com/bluez/bluez/issues/1950), [#2337](https://github.com/bluez/bluez/issues/2337), and [#2353](https://github.com/bluez/bluez/issues/2353) independently document persistent or silent generic-backend TX failures, scan/advertise conflicts, and D-Bus acceptance followed by zero RF transmission. Hardware traces also showed the ARM64 controller accepting 100 ms advertising after rejecting 20 ms. The patch serializes HCI transactions, owns deferred callbacks, validates completions and complete report batches, and derives packet dwell from the accepted interval. | `bluetooth-meshd` owns the raw-HCI bearer, command sequencing, controller completions, active bearer packets, deferred callbacks, and batched reports. ROS observes the accepted `Node1.Send` call described in the upstream false-success reports. RF scheduler correctness therefore belongs beside those daemon-owned objects. |
| `bluez-mesh-sar-queue.patch` | Upstream lower transport queues segmented work outside application ownership. SeqZero alone under-specifies ACK lookup, partial ACK progress can retain stale repetitions, and queued encrypted packets can outlive an overlay. The patch matches source/destination/subnet/SeqZero/OBO, accumulates partial bitmaps, refreshes missing segments, coalesces ACKs, and provides exact attachment-scoped handles. | `bluetooth-meshd` owns segment ACK bitmaps, encrypted network PDUs, SAR timers, Friend/OBO state, replay state, and bearer queues. The added D-Bus handles expose pending queries and cancellation before encryption. Application receipts then govern completed transfers above that daemon-owned transport state. |
| `bluez-mesh-local-pb-adv.patch` | The Mesh API defines an omitted `AddNode` `Server` as local provisioning. BlueZ 5.87 substitutes the local node's primary address and routes the request through its Remote Provisioning Server. On both test radios that self-routed link repeatedly opened and closed; direct PB-ADV completed enrollment. | Omitting `Server` is the complete D-Bus request for local provisioning. The daemon chooses and opens the provisioning bearer, so the fix belongs where BlueZ interprets that request and selects PB-ADV. |
| `bluez-mesh-joined-provisioner-keyring.patch` | A node provisioned over PB-ADV receives operational NetKey/AppKey material in its node state, while BlueZ 5.87 leaves its `Management1` keyring incomplete. `ExportKeys` then fails, blocking local vendor-model configuration and downstream provisioning. The patch copies missing local persistent keys and records AppKey Add. | BlueZ owns the plaintext keys and exposes identity and completion through `JoinComplete`; `ImportSubnet` itself requires the existing key. Propagating the provisioned key into the daemon's management keyring preserves one Mesh trust boundary and keeps Zenoh outside key distribution. |
| `bluez-mesh-reconcile-local-devkey.patch` | A stale self DevKey entry makes local Config Server traffic decrypt as remote-device-key traffic after identity reuse. The patch compares the self entry with the node's authoritative stored DevKey on attach and replaces that entry when required. | BlueZ owns both the authoritative local DevKey and its attached-node keyring. `Management1.ImportRemoteNode` enforces a disjoint remote range, placing self-entry reconciliation in the daemon's attach transaction. |
| `bluez-mesh-userspace-aes-ccm.patch` | On the ARM64 test host the pristine ELL AES-CCM known-answer test fails because host security policy disables `algif_aead`; the same vector passes through OpenSSL EVP. The patch preserves BlueZ's startup known-answer check and selects OpenSSL EVP as the AES-CCM provider. | `bluetooth-meshd` encrypts and decrypts network and application traffic around the D-Bus access-message boundary. Selecting the provider there preserves the Mesh implementation and the host's kernel-module security policy. |
| `bluez-mesh-sequence-reservation.patch` | BlueZ can transmit after an in-memory sequence increment and before that value becomes durable. A daemon or power restart may then reuse an observed sequence number and trigger peer replay protection. The patch reserves a monotonic block synchronously before attachment permits transmission and preserves the durable high-water mark at clean shutdown. | The daemon assigns network sequences to Config, control, relay, and application traffic. Durable reservation must precede that private allocator for every traffic class, which places the transaction inside BlueZ. |

The upstream audit was refreshed at `5.87-290-g73e934b` (2026-09-29). Patched
Mesh source files are identical across `5.87..73e934b`. Current master clears
`pending_settings` on adapter-command failure and leaves successful no-change
completion untreated. All eight patches apply cleanly, in package order, to
pristine 5.87 and that master revision. Upstream has yet to integrate these
changes. Package builds remain pinned to 5.87, and `git apply --check` makes any
upstream context change fail the build.

## ROS-side reliability design

Each channel builds a sorted unique ring from the local address and every fresh,
authenticated same-swarm member. The current sender selects its sorted successor
and records that destination in the transmitted unicast handoff. Receivers
follow the recorded choice, so differing membership refresh times converge on
one turn owner. The lowest current live address initializes or recovers a lost
turn. Configured probe addresses receive bounded discovery transmissions and
join the ring after an authenticated heartbeat establishes live membership.

A token holder fans the newest 60-byte topic value out as acknowledged unicast
to every live member. Every destination has independent native SAR ownership and
an application receipt. This N-peer design delivered substantially better RF
results than group-primary delivery on the target controllers. A gitignored
deterministic five-peer model test covers shuffled and duplicate membership,
every successor, membership removal, sole recovery ownership, simultaneous
initialization, and probe admission after authenticated liveness.

The example overlay sets source network retransmissions to zero. Native
lower-transport SAR selectively retransmits missing segments, while relay
retransmission supports multi-hop forwarding. An A/B count of two sent every
source PDU three times and delayed turn handoff. The configured 10 Hz source
rate and wire layouts stay constant across both tests; while the bearer is busy,
the service retains the newest value for the next available turn.

Each channel holds at most one blocking transaction and its newest waiting
value. Compact receipt queries let a receiver repeat its application receipt
after native repair. Receipts are scoped by source, AppKey, swarm, vendor opcode
and application sequence. Transfers are cancelled after receipt, expiry,
overlay replacement or application disconnect. Handles include the attachment
generation, preventing delayed cleanup from cancelling a replacement network.

Startup buffers the newest topic value until Config Server status messages
confirm attachment, AppKey binding, and local radio configuration. Those
protocol confirmations define Mesh readiness.

## Hardware evidence

Both the BlueZ daemons and ROS package compile on x86-64 and ARM64 test hosts.
Tests use `rmw_zenoh_cpp` with one local probe per UAV. This topology isolates
the measured peer payload to Bluetooth while Zenoh supplies local ROS delivery.
The gitignored probe derives every expected field from the sender and message
timestamp, validates exact timestamp, pose and twist values, counts duplicates,
and records end-to-end age and inter-delivery gaps.

Final 300-second bidirectional soaks passed in every mode with no corrupt or
duplicate ROS delivery. GATT delivered 2,537 exact samples from peer B to peer A
and 2,494 in reverse (8.453/8.312 Hz), with 0.375/0.534 s maximum gaps and
0.434/0.220 s maximum ages. Advertisement delivered 906 and 2,324 exact
samples (3.019/7.746 Hz), with 1.798/0.521 s maximum gaps and 0.136/0.176 s
maximum ages. Mesh delivered 187 exact full-size samples in each direction
(0.623 Hz), with 4.219 s maximum gaps and 2.981/2.429 s maximum ages. Every
overlay reverted to default successfully.

Randomized process-restart campaigns exercised each mode with peer A alone,
peer B alone, and both UAVs restarting at independent times. Every campaign
recovered automatically through the D-Bus configuration service with exact
payloads and zero duplicate ROS delivery. One-sided GATT gaps stayed at or
below 2.199 s and Advertisement gaps at or below 4.000 s. One-sided Mesh gaps
were 5.980 s when peer A restarted and 13.215/11.969 s when peer B, the current
turn holder, restarted. The overlapping two-sided campaign measured GATT gaps
of 2.065/1.967 s, Advertisement gaps of 3.394/1.216 s, and Mesh gaps of
17.039/15.479 s. The longer Mesh case is the bounded 12 s lost-turn safety
guard plus D-Bus reattachment while the deterministic recovery member also
restarts; delivery resumed through the existing daemon with zero operator action.

A focused post-fix regression repeated the active failure cases. A two-sided
GATT process-restart run delivered 502/502 and 459/459 exact samples, with
1.929/2.074 s maximum gaps and 0.201/0.226 s maximum ages. Advertisement
one-sided and two-sided restart runs delivered 13–35 exact samples per
direction; maximum gaps ranged from 6.171 to 9.661 s and maximum ages from
3.790 to 13.348 s. The process-start cache seeding removed the previously
observed 10,000-second stale replay. Mesh startup delivered 37/37 and 36/36
exact samples in 60 seconds, with 3.574/4.171 s maximum gaps. Its randomized
two-sided 90-second restart run delivered 46/46 and 48/48 exact samples, with
7.628/5.987 s maximum gaps and 3.131/2.477 s maximum ages. Every focused run
recorded zero corrupt samples and zero duplicate ROS delivery.

Serial-profile SSH completed in both directions through the generated PTY,
proxy, and per-link `sshd -i`. Each direction then closed and re-established
the profile and completed a second authenticated command. The TUI additionally
completed two consecutive `h` sessions on both architectures while renewing the
consumed RFCOMM connection between them.

A separate standard-use campaign kept peer B active while peer A returned to
its default configuration for eight seconds and then reapplied the same overlay.
The post-reapply windows delivered 360/372 exact GATT samples, 159/287 exact
Advertisement samples, and 23/26 exact Mesh samples, with no corruption or
duplicates. This also covered a peer remaining early while the other joined
late. All three directed mode changes and all three reapplications succeeded.

The immediately preceding configuration, with three transmissions for every
source network PDU and a 3 s lost-token deadline, failed a 60-second gate:
peer B to peer A reached a 7.489 s gap while the lowest address reclaimed during a
healthy successor's SAR/ACK drain. The reverse direction reached 2.968 s. This
A/B result motivated zero blanket network retransmissions and a 12 s
lost-token-only recovery deadline.

A group-primary A/B run delivered two samples per direction after 40–45 seconds,
with roughly 17–21 s gaps and 18–20 s ages. That result disqualified it from the
reliability gate and established acknowledged unicast as the N-peer data path.

Earlier patched-stack validation measured 26-byte advertisement messages at
4.60–4.96 Hz with gaps up to 1.30 s, and 60-byte GATT messages at
7.35–8.60 Hz with gaps up to 0.30 s. Six directed transitions between the three
modes, pending-connection handoff, one-sided overlay reset, adapter property
checks, and local ASan/UBSan probes passed. Those runs supply additional
regression evidence; the current long/restart campaign supplies acceptance.

The advertising-update A/B gate now covers repeated standard
`LEAdvertisingManager1` unregister/register operation, a 300-second soak,
one-sided restarts, two-sided restarts, and one-sided overlay reapplication.
Every case passed through the distribution BlueZ advertising D-Bus path, which
places advertisement update behavior entirely in the ROS package.

The hardware campaign used two physical UAV radios. A deterministic five-peer
model covers N>2 membership, successor choice, single recovery ownership,
asymmetric membership views, removal, and probe admission. A third radio is
required for N>2 and multi-hop RF characterization. This stated hardware scope
keeps the patch audit tied to measured evidence from the available controllers.

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
