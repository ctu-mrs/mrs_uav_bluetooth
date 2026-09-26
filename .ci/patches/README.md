# BlueZ Mesh radio fixes

The local patches against BlueZ 5.87 are applied by
`build_bluez_package.sh` to produce `mrs-bluez 5.87`. They are not represented
as upstream-accepted patches.

`bluez-mesh-single-tx-worker.patch` fixes burst scheduling in the generic HCI
backend.

The generic HCI backend used only `pvt->tx` to decide whether to schedule an
idle worker. A segmented group send enqueues a burst before that callback runs,
and later enqueues can also occur between HCI transactions while a TX timer
still owns the queue. Multiple workers can then replace the shared current
packet before its asynchronous advertising command chain completes.

The fix schedules a worker only when the current packet, timer, and queue are
all empty. Network encryption, replay protection, segmentation, keys, and
controller selection are unchanged. The package build fails if the source
context no longer matches. Do not silently drop the patch when updating BlueZ.

`bluez-mesh-local-pb-adv.patch` keeps a local `AddNode` request on the direct
PB-ADV bearer. BlueZ 5.87 otherwise routes it through its own Remote
Provisioning Server even when the caller did not request a remote server.
During the NUC/RPi trial, both radios heard the relevant provisioning
advertisements, but the self-routed link repeatedly opened and closed without
completing. The direct path removes that unnecessary local loop. With the
keyring fix below, NUC provisioned RPi and both exchanged timestamped
topic over the same Mesh.

`bluez-mesh-joined-provisioner-keyring.patch` seeds management NetKey, AppKey,
and local DevKey entries from a provisioned node's own state. BlueZ normally
leaves that keyring empty on a newly joined node. Without it, `ExportKeys`
fails and the joined host cannot configure its local model or provision later
hosts. Existing entries are preserved, so a daemon restart repairs a missing
keyring without changing an established identity.

On NUC (x86_64) and RPi (aarch64), unpatched raw HCI delivered heartbeats
but repeatedly failed bounded full-topic probes. Patching the sender restored
delivery. Patching both senders restored bidirectional delivery.

The three patches were applied and compiled on both host architectures. Their
order was also checked against a clean BlueZ 5.87 tree. The host trial used
the patched daemons directly. A fresh binary package install was not part of
that trial.

Upstream context:
- [generic HCI implementation](https://github.com/bluez/bluez/blob/5.87/mesh/mesh-io-generic.c)
- [related persistent TX-stall report](https://github.com/bluez/bluez/issues/2353)

The reported upstream stall has a different signature and does not prove the
cause or validate this fix. Our evidence is the source analysis and two-host
before/after tests. Broader controller and long-duration soak testing remains
necessary.
