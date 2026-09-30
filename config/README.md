| Parameter | What it controls | Allowed values or range |
| --- | --- | --- |
| node_topics_prefix | Root of this UAV's Bluetooth ROS topics. {hostname} inserts the local name. | ROS topic prefix string. |
| scan_publish_period | How often nearby-device information is published. | Number of seconds; use > 0. |
| time_update_period | How often connected clocks are updated. | Number of seconds; use > 0. |
| wifi_refresh_period | How often available Wi-Fi networks are checked. | Number of seconds; use > 0. |
| auto_connect_period | Delay between automatic connection checks. | Number of seconds; use > 0. |
| discoverable_timeout | How long this UAV remains visible. | Integer 0..4294967295 seconds; 0 means indefinitely. |
| auto_pair | Automatically approve permitted pairing requests. | true or false. |
| auto_accept_pairing | Older name for auto_pair, used only when auto_pair is absent. | true or false. |
| auto_trust | Remember a permitted paired UAV as trusted. | true or false. |
| enable_time_service | Share clock information over a connection. | true or false. |
| enable_wifi_service | Share Wi-Fi choices over a connection. | true or false. |
| enable_serial_port_profile | Offer a Bluetooth serial link for SSH. | true or false. |
| serial_port_channel | Channel used by the serial link. | Integer 1..30. |
| serial_sshd_path | SSH server program launched for the serial link. | Executable path string. |
| enable_mesh | Give the radio to Mesh networking. | true or false; Mesh uses its own radio session. |
| mesh_auto_attach | Reopen this UAV's saved Mesh network at startup. | true or false. |
| mesh_auto_join | Join a Mesh network when instructed to do so. | true or false; cannot be combined with mesh_auto_create_network. |
| mesh_auto_create_network | Create a Mesh network when instructed to do so. | true or false; cannot be combined with mesh_auto_join. |
| mesh_provisioner | Allow this UAV to add new Mesh members. | true or false; automatic provisioning enables it. |
| mesh_auto_configure_relay | Configure forwarding for other Mesh members. | true or false; automatic provisioning enables it. |
| mesh_default_ttl | Maximum forwarding hops for a sent Mesh packet. | 0 or integer 2..127; 0 keeps the packet local to one radio hop. |
| mesh_relay_retransmit_count | Extra broadcasts when forwarding someone else's packet. | Integer 0..7. |
| mesh_relay_retransmit_interval_steps | Spacing between forwarded broadcasts. | Integer 0..31; each step is 50 ms, starting at 50 ms. |
| mesh_network_retransmit_count | Extra broadcasts of this UAV's own Mesh packet. | Integer 0..7. |
| mesh_network_retransmit_interval_steps | Spacing between own-packet broadcasts. | Integer 0..31; each step is 10 ms, starting at 10 ms. |
| mesh_swarm_auto_provisioning | Automatically create or join a private Mesh with listed UAVs. | true or false; requires enable_mesh and a nonempty peer_whitelist. |
| mesh_swarm_id | Identifier for the logical group sharing the Mesh. | Integer 1..65535; identical on members. |
| mesh_swarm_participating | Whether this UAV currently shares group data. | true or false. |
| mesh_swarm_state_path | File that remembers group participation. | Writable file path; {hostname} is expanded. |
| mesh_provisioner_preference | Which available UAV should add new members first. | ordered or a hostname from peer_whitelist. |
| mesh_swarm_startup_grace | Wait before forming a new private Mesh. | Finite seconds > 0. |
| mesh_swarm_heartbeat_period | Interval between member check-ins. | Finite seconds > 0. |
| mesh_swarm_provisioner_timeout | Silence before choosing another member to provision. | Finite seconds > twice mesh_swarm_heartbeat_period and <= 63. |
| mesh_swarm_group_address | Shared destination for Mesh group messages. | Integer 0xc000..0xfeff in automatic mode. |
| mesh_swarm_network_index | Mesh network key slot. | Integer 0..4095; identical on members. |
| mesh_swarm_app_key_index | Mesh application key slot. | Integer 0..4095; identical on members. |
| mesh_unicast_cursor_path | File remembering assigned member addresses. | Writable file path; {hostname} is expanded. |
| mesh_device_uuid | Fixed identity of a manually managed Mesh node. | Empty for automatic mode; otherwise exactly 32 hexadecimal digits, with optional hyphens. |
| mesh_token | Saved Mesh attachment token. | Integer 0..18446744073709551615; 0 lets the service load its stored token. |
| mesh_token_path | File holding the Mesh attachment token. | Writable file path; {hostname} is expanded. |
| mesh_company_id | Identifier for this Mesh application's message family. | Integer 0..65535; identical on members. |
| mesh_product_id | Product identifier advertised by a Mesh node. | Integer 0..65535. |
| mesh_version_id | Version identifier advertised by a Mesh node. | Integer 0..65535. |
| mesh_crpl | Number of replay entries supported by a Mesh node. | Integer 1..65535. |
| mesh_vendor_model_id | Model identifier for this Mesh application's messages. | Integer 0..65535; identical on members. |
| mesh_next_unicast | Next address used for manual Mesh assignment. | Integer 1..32767. |
| mesh_agent_capabilities | External confirmation methods available during manual Mesh joining. | List drawn from blink, beep, vibrate, out-numeric, out-alpha, push, twist, in-numeric, in-alpha, public-oob, static-oob; [] for automatic setup. |
| mesh_agent_oob_info | Where a manual joining code or identity can be found. | List drawn from other, uri, machine-code-2d, barcode, nfc, number, string, on-box, in-box, on-paper, in-manual, on-device; [] normally. |
| mesh_agent_static_oob | Prearranged secret for manual Mesh joining. | Empty or exactly 16 bytes (0..255 each), as a YAML list or space/comma-separated scalar. |
| mesh_agent_private_key | Private key for manual Mesh joining. | Empty or exactly 32 bytes (0..255 each), as a YAML list or space/comma-separated scalar. |
| mesh_agent_public_key | Public key for manual Mesh joining. | Empty or exactly 64 bytes (0..255 each), as a YAML list or space/comma-separated scalar. |
| mesh_agent_numeric_oob | Number entered during manual Mesh joining. | Integer 0..99999999. |
| mesh_agent_uri | External setup-information address for manual Mesh joining. | String; empty when unused. |
| gatt_profile_uuids | Optional service identities used to find connectable UAVs. | List of 128-bit UUID strings; [] disables this filter. |
| auto_connect_enable | Start GATT connections to permitted nearby UAVs. | true or false. |
| peer_whitelist | UAVs allowed to exchange data. | List of unique, nonempty hostnames; [] accepts transport-approved peers. Automatic Mesh needs numbered UAV names, including this UAV, with unique numbers 1..32767. |
| auto_connect_pattern | Pattern matching UAV names for automatic connections. | Regular-expression string. |
| peer_connection_timeout | Idle time before closing an unused GATT connection. | Number of seconds; use > 0. |
| wifi_netplan_config_path | System network file changed after an approved Wi-Fi choice. | File path string. |
| allowed_wifi_networks | Wi-Fi names a peer may ask this UAV to select. | List of network-name strings; [] allows none. |
| status_report_period | Interval between service status reports. | Number of seconds; 0 disables reports. |
| log_topic_enable | Publish detailed log messages as a ROS topic. | true or false. |
| expire_connections_with_overlay | Close overlay-created connections when that overlay ends. | true or false. |
| overlay_keepalive_topic_suffix | Topic name used to keep a temporary overlay active. | Nonempty ROS topic suffix string. |
| verbose_log_file | Optional detailed log destination. | File path string; empty disables file logging. |
| advertise_mode | Whether broadcasts invite connections or carry topic data. | peripheral or broadcast. |
| advertise_size | Radio broadcast format. | legacy or extended; extended requires radio support. |
| advertise_update_strategy | How changed broadcast bytes reach the radio. | property or reregister; property is recommended for changing topic data. |
| advertise_local_name | Name included in broadcasts. | String; {hostname} expands; empty omits it. |
| advertise_discoverable | Whether a broadcast asks nearby devices to discover this UAV. | true, false, or omitted for BlueZ default. |
| advertise_includes | Extra broadcast fields requested from the radio. | List of tx-power, appearance, or local-name; [] for none. |
| advertise_service_uuids | Service identities included in the main broadcast. | List of UUID strings; [] for none. |
| advertise_solicit_uuids | Service identities requested in the main broadcast. | List of UUID strings; [] for none. |
| advertise_manufacturer_data | Company-coded byte fields in the main broadcast. | Map: company key 0..65535 to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_service_data | Service-coded byte fields in the main broadcast. | Map: UUID string to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_data | Explicit raw fields in the main broadcast. | Map: field key 0..255 to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_scan_response_service_uuids | Service identities returned when a scanner asks for more detail. | List of UUID strings; [] for none. |
| advertise_scan_response_manufacturer_data | Company-coded bytes returned to a scanner. | Map: company key 0..65535 to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_scan_response_solicit_uuids | Service identities requested in a scan response. | List of UUID strings; [] for none. |
| advertise_scan_response_service_data | Service-coded bytes returned to a scanner. | Map: UUID string to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_scan_response_data | Explicit raw fields returned to a scanner. | Map: field key 0..255 to bytes 0..255, as a list or space/comma-separated scalar. |
| advertise_appearance | Optional numeric device-category hint. | Integer 0..65535 or omitted. |
| advertise_duration | How long one advertising instance stays active. | Integer 0..65535 seconds or omitted for BlueZ default. |
| advertise_timeout | When BlueZ removes an advertising instance. | Integer 0..65535 seconds or omitted for BlueZ default. |
| advertise_min_interval | Shortest interval between broadcast packets. | Integer 0..4294967295 ms or omitted; practical range depends on the radio. |
| advertise_max_interval | Longest interval between broadcast packets. | Integer 0..4294967295 ms or omitted; must be at least advertise_min_interval. |
| advertise_tx_power | Requested broadcast strength. | Integer -32768..32767 dBm or omitted; radio support may narrow this. |
| advertise_extra_data_topic | ROS topic supplying optional raw broadcast bytes. | ROS topic string; empty disables it. |
| pairing_agent | BlueZ pairing interaction this UAV can perform. | NoInputNoOutput, DisplayOnly, DisplayYesNo, KeyboardOnly, or KeyboardDisplay. |
| enable_server | Offer local Bluetooth data/services to nearby devices. | true or false. |
| enable_scan | Observe nearby broadcasts. | true or false. |
| scan_mode | Bluetooth radio types included in a device scan. | le, bredr, or auto. |
| shared_topics | Topic bridges enabled by this overlay. | List of bridge maps; [] shares none. All entries must use one transport mode per overlay. |
| shared_topics[].name | Human-readable label for a bridge. | Nonempty string; defaults to its topic path. |
| shared_topics[].transport | Radio mode used by this bridge. | gatt, advertisement, or mesh. |
| shared_topics[].mode | Which direction this UAV shares the topic. | export, import, or both. |
| shared_topics[].payload_format | How a ROS message becomes bytes. | struct for selected fields, raw for byte-array messages, or ros2 for a complete serialized message. |
| shared_topics[].export_topic | Local ROS source topic. | Nonempty ROS topic path; {hostname} expands. |
| shared_topics[].import_topic_suffix | Name below this UAV's peer-topic prefix for received data. | ROS topic suffix string; defaults to the source path without its UAV prefix. |
| shared_topics[].message_type | ROS message type of this bridge. | Installed type string such as nav_msgs/msg/Odometry. |
| shared_topics[].rate_hz | Maximum offered source update frequency. | Number >= 0 Hz; 0 offers every source update. Delivered rate also depends on the radio. |
| shared_topics[].key | Legacy topic identity label accepted by the parser. | Nonempty string; the bridge identity is derived from the normalized source topic. |
| shared_topics[].channel_id | Wire identifier for a framed advertisement or Mesh bridge. | Integer 1..65535, unique per transport; omit for GATT and bare advertisement. |
| shared_topics[].framing | Whether advertisement packets include a channel header. | channel or bare; bare requires exactly one advertisement bridge and no channel_id. |
| shared_topics[].members_encode | Values placed into a struct payload, in wire order. | Nonempty list for struct; raw may omit it for a byte array; ros2 does not need it. |
| shared_topics[].members_encode[].target | Encoded value name or direct ROS field path. | Nonempty, unique field-path/name string. |
| shared_topics[].members_encode[].type | Numeric representation stored on the wire. | bool; signed or unsigned 8, 16, 32, 64-bit integer; float32 or float64. Aliases: boolean, byte, octet, char, float, double. |
| shared_topics[].members_encode[].expression | Calculation of a value from ROS message fields. | Optional expression string; omit for direct field copy. |
| shared_topics[].members_decode | Rules that rebuild ROS fields from encoded values. | List of target/expression maps; [] when direct-copy fields suffice. |
| shared_topics[].members_decode[].target | ROS field to fill after decoding. | Nonempty ROS field path string. |
| shared_topics[].members_decode[].expression | Calculation from decoded wire values. | Nonempty expression string. |
| shared_topics[].destination | Fixed Mesh address instead of automatic reliable group delivery. | Integer 0..65535; omit for automatic Mesh peer delivery. |
| shared_topics[].app_key_index | Mesh application key slot for this topic. | Integer 0..4095. |
| shared_topics[].element_index | Mesh element carrying this topic. | Integer 0..255. |
| shared_topics[].force_segmented | Request acknowledged multi-packet Mesh delivery. | true or false. |
| shared_topics[].vendor_opcode | Message-family code for this Mesh topic. | Integer 0xc0..0xff; 0xc0 with the group company ID is reserved by automatic setup. |
| shared_topics[].company_id | Company code paired with the Mesh message-family code. | Integer 0..65535; identical on all receivers. |
