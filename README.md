# 🌐 In-Network Processing for Real-Time Point-Cloud Streaming
> **Experimental Thesis Project — Sapienza University of Rome**<br>

> A "DPDK" / "SFC" Data-Plane architecture for volumetric transport, in-path geometric processing, workload-aware source control, "GPU" projection, persistent "H.265" delivery, reconstruction, & quality evaluation.

### 👥 Academic Information
**Author:** Leonardo Chiarparin ( Student ID: **2016363** )<br>

**Thesis Supervisor:** Professor Marco Polverini<br>

**Degree Programme:** Engineering in Computer Science<br>

**Institution:** **Sapienza University of Rome**

---

## 📌 Realization Status

This repository serves as an experimental research platform rather than a production-ready "Service Function Chaining" ( "SFC" ) framework. The present snapshot implements & validates the complete `Camera`-to-`User` volumetric chain, the reverse workload-driven "Temporal" control path, the independent `User`-originated "Pose" path, the asynchronous browser bridge, & complementary runtime / fidelity measurements.

| Node | Condition | Responsibility |
|---|---|---|
| `Camera` | Validated | "DPDK"-native point-cloud source, warm-mode file acquisition, frame packetisation, absolute scheduling, temporal selection, bounded local Tx resubmission, & source telemetry |
| `SFF1` | Validated | "Geometry-Aware Classifier" ( "GAC" ) implementing packet-progressive spatial aggregation, exact frame-completing radius evaluation, final projection-metadata derivation, experimental "NSH" metadata insertion, & "Temporal" decapsulation directed to `Camera`; the geometric work is compile-time selectable through `NETWORK_PROCESSING` for the fair placement ablation |
| `SFF2`<br>( Route 0 ) | Validated | Stateful proxy for `SFF1` -> `Encoder`; validates the aware envelope, preserves proxy state, strips service metadata, & forwards plain geometry-bearing application datagrams |
| `Encoder` | Validated | "SFC"-unaware frame assembly, geometry-offload consumption or local fallback, workload-driven "Temporal" regulation, "CUDA" projection, persistent "FFmpeg" / "NVENC" encoding, "MPEG-TS" attribution, & optional post-stream "luma"-quality evaluation |
| `SFF2`<br>( Route 1 ) | Validated | Proxy-maintained `Encoder` -> `Decoder` transition, compressed-media integrity accounting, & advancement of the primary service state |
| `Decoder` | Validated | "SFC"-unaware persistent hardware-accelerated "H.265" decoding, occupancy erosion, geometric reconstruction, dynamic pose application, output packetisation, & reconstruction telemetry |
| `SFF2`<br>( Route 2 ) | Validated | Stateful `Decoder` -> `SFF3` transition; advances the retained primary state, re-imposes the base service envelope, & records route-specific reconstructed-point telemetry |
| `SFF3` | Validated | Final aware primary-path boundary; validates / removes base service metadata before `User`, while classifying & encapsulating reverse 24-byte "Pose" commands |
| `User` | Validated | Final reconstructed-frame reassembly, shared-memory publication, asynchronous "Pose" dispatch with three-frame re-presentation until returned-state confirmation, command / browser acknowledgment telemetry, in-memory quality capture, & terminal synchronization |
| "Python" / "WebSocket" bridge | Validated | Asynchronous latest-frame publication with one frame in flight per peer, shared-memory control exchange, browser acknowledgments, & independent "HTTP" serving |
| `Three.js` viewer | Validated | Dynamic point-cloud rendering, keyboard / button interaction, command-to-photon acknowledgment, & on-demand scene refresh |
| `Gauge` | Validated | Post-"EOS" serial geometric assessment using pose reversal, robust "ICP", statistical outlier filtering, symmetric nearest-neighbour metrics, & direct telemetry merging |

The **validated primary route**, designated as **"Main"**, is:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 ) -> Decoder -> SFF2 ( Route 2 ) -> SFF3 -> User
```

Control mechanisms are kept in two independent logical service paths rather than being consolidated into a monolithic feedback packet:

```
"Temporal" : Encoder -> SFF2 -> SFF1 -> Camera
"Pose"     : User -> SFF3 -> SFF2 -> Decoder
```

The "Temporal" loop is fully operational. `Encoder` derives a requested `temporal_skip`, `SFF2` classifies the plain control datagram & imposes the corresponding service state, `SFF1` validates / removes the envelope, & `Camera` applies the reflected factor before subsequent source-frame admission. All three `B0` repetitions, `BQ`, & the completed `OQ-OFF` fidelity control remain at `current_skip = 1`. The runtime sensitivity campaign exercises the same reverse path under two distinct overload mechanisms: the geometry-placement ablation moves the complete frontier back to `Encoder`, while `P40` records three `skip 1 -> 2 -> 1` cycles under elevated point-datagram pressure. The final fair `O-OFF` protocol disables the corresponding `SFF1` geometry work as well; both completed reruns therefore change placement rather than duplicating the same computation on both sides of the boundary.

The "Pose" cycle is likewise functional. Browser-originated `yaw`, `pitch`, & `zoom` modifications are written into the `User` control mapping, dispatched as a plain 24-byte payload by the native `User`, encapsulated by `SFF3` as `SPI 300 / SI 255`, stripped by `SFF2`, & consumed by `Decoder`. `Decoder` applies the most recent accepted pose during reconstruction; the corresponding values return with the reconstructed frame & are correlated by `User` with command identifiers & browser render acknowledgments.

> **Repository Note:** The final topology no longer depends upon "Open vSwitch" or an "OVS"-"DPDK" "PMD". Native components are attached directly through explicit `/tmp/sfc-*` "vhost-user" / "virtio-user" adjacencies. This removes an intermediary switching stage & releases the logical core previously dedicated to the virtual switch for application-level placement.

> **Validation Scope:** Runtime & fidelity measurements remain methodologically separate. The common runtime configuration has been repeated three times as `B0`, while `BQ` & `OQ-OFF` provide capture-enabled objective "luma" / reconstructed-geometry controls after stream termination. All five complete reference / fidelity executions sustain 300 / 300 frames at `current_skip = 1` & approximately `30.0 frames / s`; the runtime sensitivity campaign is interpreted separately whenever a changed condition alters cadence, temporal admission, or terminal completeness.

> **Current Measurement Snapshot:** The updated `outcomes(5)` campaign makes the in-network placement result explicit. The best complete runtime baseline is `B0-R1` with `211.352 ms` all-frame / `211.222 ms` steady Camera -> `User` latency against the supplied `341.785 ms` reference frontier ( `38.16 %` reduction ). The fair runtime `O-OFF` attempts remain complete at `300 / 300` but move the workload back to `Encoder`: `Encoder.active_process_ms` rises from `17.684 ms` in `B0-R1` to `54.335 / 26.081 ms`, while the steady workload-ratio median rises from `0.165` to `0.419 / 0.391`. Active `P40` admits `270` frames & preserves the complete admitted set through `SFF2 Route 2`; the residual `246 / 270` terminal completion is localised to the measured `SFF3` -> `User` Tx backpressure frontier, rather than to hidden upstream frame loss.

---

## 🎯 1. Project Motivation & Research Objective

The central thesis objective is to take the volumetric point-cloud pipeline developed in the reference work & redistribute its processing responsibilities over an explicit in-network service graph. The contribution is a concrete "DPDK" / "SFC" realisation of packet-visible volumetric processing: coarse application blocks are decomposed into independently measurable, placeable, & steerable functions, while reusable geometry is computed in-path before the frame-completing `Encoder` boundary & the same 300-frame / 30 frames / s operating objective is preserved.

This project investigates the feasibility of migrating selected functions within a real-time volumetric streaming pipeline from conventional application-level microservices to direct execution during packet traversal through a "DPDK"-based "SFC".

The objective is therefore practical rather than only architectural: the forwarding path is made computationally active, the source can be regulated before packetisation, the `Encoder` consumes an already resolved geometry frontier in the nominal case, & every conclusion is checked against route-level completion, queue / backlog telemetry, workload ratios, terminal latency, & fidelity measurements.

> **Real-Time Scope:** Within this repository, **real-time-oriented** denotes preservation of the absolute `TARGET_FPS = 30` source schedule, bounded queue / backlog behaviour, & complete frame semantics throughout the nominal 300-frame operating condition. It does **not** denote a hard real-time deadline guarantee: multiple frames can coexist within the persistent "codec" pipeline, native frame-ready latency remains an independently measured end-to-end quantity, & browser "Command-to-Photon" remains a render-completion proxy rather than a physical display-latency measurement.

### 1.1 Why "DPDK" & "UDP" Are Design Requirements


"DPDK" is employed because the experiment necessitates more than aggregate throughput. Service functions require the capability to inspect & modify the packet envelope directly, retain per-frame state, perform incremental computations while traffic is in flight, observe local queue acceptance, & account separately for active `rte_eth_tx_burst()` execution versus wall-clock backpressure. A conventional kernel-socket path would obscure portions of the packet lifecycle behind socket queues, scheduler wake-ups, generic buffering, & transport-stack policies. While valuable for general-purpose applications, such mechanisms would blur the principal experimental boundaries.

"UDP" is selected for a more specific reason than the generic assertion that an unreliable transport is simply "faster" than "TCP". In the present system, **the transport is part of the experimental protocol contract**. "RFC 768" defines "UDP" as a transaction-oriented datagram service with a minimum 8-byte transport header, while "RFC 8085" characterises it as a minimal message-passing transport that avoids connection establishment / teardown & maintains little transport-layer end-system state. "TCP", conversely, provides a reliable, in-order byte stream through sequence / acknowledgment state, send & receive windows, retransmission, congestion control, & a connection state machine. Those properties are valuable when the application requires stream reliability; they are stronger than the semantics required by the present packet-oriented service paths.

The distinction is especially important for the two control loops. `temporal_payload` & `pose_payload` are not fragments of a command stream whose every historical transition must eventually be executed. They represent **absolute desired state**:

```text
"Temporal" -> { frame reference, timestamp, requested skip }
"Pose"     -> { timestamp, absolute yaw, absolute pitch, absolute zoom }
```

Accordingly, the reliability objective is **latest-state convergence under eventual successful delivery**, rather than reliable delivery of every intermediate byte or command event. `Encoder` re-presents an unresolved `temporal_skip` every `RETRY_FRAMES = 3` observations until the returned `Camera` stream reflects the requested factor. `User` analogously re-presents the same unresolved "Pose" every three completed frames until the requested stance is visible in the returning reconstructed stream. "Pose" retries preserve the original timestamp; `Decoder` ignores an already consumed timestamp & rejects older timestamps, making duplicate or delayed copies semantically harmless. A newer browser request supersedes the preceding unresolved request at `User`. The application therefore supplies the reliability property actually required by the control plane: **latest-state confirmation with duplicate / stale-state suppression**, conditional upon at least one subsequent re-presentation reaching the destination while the stream remains active.

This distinction is more than a reformulation of head-of-line blocking. Carrying these controls through "TCP" would impose a different semantic contract: the transport would reliably preserve an ordered history of intermediate commands even when a more recent absolute state has rendered those commands obsolete. The application would still require its own coalescing / supersession logic to recover latest-state semantics. Furthermore, "TCP" exposes a byte stream rather than application message boundaries. A native `SFF` could no longer classify one command through the current deterministic `ingress port + exact payload length + protocol fields` rule without introducing a stream-framing / reassembly layer or terminating the connection. "UDP" instead maintains a one-command / one-datagram correspondence, allowing `SFF2` & `SFF3` to classify, encapsulate, validate, & decapsulate a complete control entity without transport reassembly.

The same uniformity applies to the primary path. Every native component already constructs & validates "Ethernet" / "IPv4" / "UDP" envelopes directly; raw geometry, compressed media, reconstructed points, "Temporal", & "Pose" therefore share one packet-processing model. Replacing only the vital controls with "TCP" would either require a kernel off-ramp or the introduction of an additional user-space "TCP" implementation with its own connection control blocks, queues, timers, acknowledgment processing, retransmission state, & stream parsing. Either alternative would introduce a qualitatively different software subsystem into the very data-plane experiment being measured.

The byte saving is real but is treated as a **secondary, lower-bound consequence**, not as the sole justification. For an identical application payload, the minimum "TCP" header is 20 B while the current "UDP" header is 8 B. Thus, before considering "TCP" options or acknowledgment-only traffic, each control data segment carries at least 12 additional transport bytes under "TCP"; Section 5.5 quantifies the resulting packet-size differences for the exact 16-B & 24-B project commands. A persistent "TCP" connection would of course amortise connection establishment & can employ `TCP_NODELAY`; therefore the project does **not** claim that every command would incur a fresh handshake or that "TCP" is intrinsically unsuitable for low-latency control. The choice instead follows from a tighter correspondence between the required state semantics & a datagram-oriented, directly inspectable "DPDK" path.

This architecture also avoids treating "UDP" as implicitly reliable. Data correctness is rendered observable through explicit frame / packet identifiers, point / media counters, integrity percentages, protocol field validation, redundant "End-of-Stream" ( "EOS" ) signalling, & bounded local resubmissions when a Tx ring temporarily accepts zero elements. Control correctness is established separately through returned-state confirmation, monotonic command metadata, & application-level re-presentation.

Finally, the deployment scope matters. "RFC 8085" correctly requires congestion-safe behaviour for "UDP" applications intended for the general Internet. The present implementation is instead a **closed, single-administrative-domain experimental environment** composed of six explicit local "vhost-user" / "virtio-user" adjacencies. Source production is isochronously bounded at `TARGET_FPS = 30`, local overload is directly observable through "DPDK" queue telemetry, & the workload-driven "Temporal" loop can reduce source admission. This restricted applicability is part of the specification; the current design must not be generalised into a claim that the same zero-checksum / no-general-Internet-congestion-control configuration is appropriate for unrestricted Internet deployment.

**Accordingly**, "DPDK" + "UDP" provide the project with:

```text
explicit packet ownership & lifetime
one application message = one inspectable datagram
fixed-width service-path classification without stream reassembly
in-place service-header manipulation
no native "TCP" connection-control state in the SFF data path
latest-state control semantics with application-level confirmation
observable local queue backpressure
frame-aware loss / integrity accounting
"MTU"-bounded packetisation without "IP" fragmentation
uniform transport parsing across "Main" / "Temporal" / "Pose" paths
source-level regulation rather than hidden transport flow-control policy
```

"DPDK" + "UDP" are therefore concrete execution requirements of the measured implementation rather than an abstract transport preference. Every retained "Main" / "Temporal" / "Pose" execution uses the same directly inspectable datagram model for packet ownership, classification, in-place service-state manipulation, local queue-pressure telemetry, & returned-state confirmation. The fair `O-OFF` experiment keeps this packet-processing substrate unchanged & isolates the placement of the geometric frontier itself.

### 1.2 Selecting Computation for the Data Plane

The design delineates operations according to their underlying mathematical dependency structures.

Operations that are **mathematically reducible, incrementally composable, & element-progressive** present as natural candidates for in-network execution ( "GAC" ):

```
sum_x, sum_y, sum_z
min_x, min_y, min_z
max_x, max_y, max_z
active_point_count
```

The reduction structure is mathematical rather than a claim of arbitrary floating-point bit invariance. Component-wise extrema are directly composable, while coordinate sums are accumulated in a fixed packet order & remain subject to ordinary finite-precision rounding / compiler transformation. The source revision, packet order, host compiler profile, & `-ffast-math` setting are therefore retained as reproducibility conditions rather than treating floating-point addition as bit-associative.

From these running quantities, `SFF1` extracts significant metrics, including:

```
C_N = ( 1 / N ) * sum_i( p_i )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

where `C_N` denotes the centroid of the observed points, while `E_N` & `B_N` represent the current axis-aligned extent & bounding-box centre, respectively.

Certain geometric entities, such as the maximum outer radius, cannot achieve exactitude prior to frame completion:

```
max_r = max_i || p_i - C_final ||_2
```

This metric is contingent upon `C_final`, which remains undetermined until the frame is entirely received. "GAC" therefore maintains a compact frame-local `XYZ` workspace, executing the initial aggregation pass while packets are actively forwarded, & performs the precise distance calculation only upon ascertaining the final population. The resultant data is then appended to the scene-completing packet.

This limitation is not merely a deferred optimisation resolvable by transmitting an alternative packet earlier. Pre-computing the accurate centroid-dependent boundary at the `Camera`, within an offline stage, or via another processing element would shift the computation outside the data-plane locale under experimental evaluation. Likewise, the final transformed projection frontier cannot be materialised until the final centroid & exact `max_r` have become available at the frame-completion barrier. Thus, the implementation rigorously distinguishes between **continuous in-path information** & **frame-global information mathematically unavailable until a specific barrier is breached**.

Once that barrier is reached, `SFF1` now completes the geometric offload by deriving `final_scale`, transformed extents, projected bounding-box centre, & `global_scale`. Procedures that still require final reconstruction parameters, "GPU" visibility, dynamic stance, or "codec" state remain downstream:

```
pose transformation
6-view projection
visibility / depth conflict resolution
"Geometry" / "Texture" / "Occupancy" "Atlas" packing
"I420" generation
"H.265" / "NVENC" compression
```

The `Encoder` operates neither as a conventional isolated application nor as a purely stateless data-plane function. It retains a frame-completion boundary prior to projection, yet it incrementally performs packet-by-packet conversion & placement, consumes progressive / final geometric metadata, cooperatively services "DPDK" during "CPU" / "GPU" workloads, & decouples "codec" input via a writer queue. Complete validated frames therefore enter projection with the final geometric frontier already resolved by `SFF1`, while analytical / local fallbacks remain available for partial or invalid upstream states. It functions as a **hybrid frame-aware elaboration node** situated between application-level semantics & data-plane-oriented incremental execution.

### 1.3 Connection with the Reference Pipeline


This methodical approach is informed by the application-level architecture documented in the reference thesis upon which the present investigation builds. The reference implementation is already distributed across four containerised services — `Camera`, Flow Controller, `Encoder`, & Client — & should therefore not be inaccurately described as a single-process monolith. The relevant limitation is **coarse deployment granularity**: its `Encoder` & Client each remain internally monolithic placement blocks even though they combine operations with markedly different "CPU", "GPU", network, & "codec" requirements. The reference thesis itself identifies their decomposition into independently deployable functions as a natural next step for service orchestration.

The repository materialises that change of paradigm. It preserves the fundamental unidirectional volumetric path & independent reverse controls, while reformulating transport, service steering, placement boundaries, & control ownership around an explicit software Data Plane:

```text
"DPDK"-native packet I / O
direct "virtio-user" / "vhost-user" adjacencies
explicit "SFC" topology without an intermediate "OVS" Data Plane
"NSH"-style "SPI" / "SI" steering
in-place geometric aggregation
stateful proxying around "SFC"-unaware applications
"GPU" projection & reconstruction
persistent "FFmpeg" / "NVENC" encoding
persistent "FFmpeg" / "NVDEC" decoding
frame-aware "MPEG-TS" attribution
workload-driven "Temporal" source regulation
independent User-originated "Pose" control applied at Decoder
asynchronous latest-frame browser delivery
per-node native telemetry
post-stream objective quality assessment
```

The architectural contrast is therefore concrete:

| Dimension | Reference Application Pipeline | Current "SFC" / "DPDK" Pipeline |
|---|---|---|
| Primary deployment graph | 4 coarse services: `Camera`, Flow Controller, `Encoder`, Client | 7 native placement units: `Camera`, `SFF1`, `SFF2`, `Encoder`, `Decoder`, `SFF3`, `User` |
| Raw upstream transport | Persistent "TCP" through `Camera -> Flow Controller -> Encoder` | Direct packet-oriented "UDP" / "DPDK" adjacencies |
| Encoded-media transport | "UDP" from `Encoder` to Client | "UDP" throughout Route 1 with explicit `SFF` proxy state |
| "Temporal" ownership | Client-originated relative control applied by a downstream Flow Controller | `Encoder`-derived absolute skip applied at `Camera` before source packetisation |
| "Pose" ownership | Client -> `Encoder`; new pose is reflected through re-projection & subsequent encoding | `User` -> `Decoder`; pose is applied during reconstruction without forcing Encoder re-projection |
| Control transport | Two persistent "TCP" feedback sockets | Two fixed-width "UDP" service paths with application-level convergence confirmation |
| Service state | Implicit in application connections / node order | Explicit `SPI` / `SI` state across aware boundaries & proxies |
| Unaware functions | Not an "SFC" concern | `Encoder` & `Decoder` intentionally remain "SFC"-unaware behind `SFF2` proxying |
| Placement granularity | `Encoder` & Client must be placed as whole blocks | Geometry service, encoding, decoding / reconstruction, terminal delivery, & `SFF` boundaries expose separate scheduling / placement points |

This decomposition increases graph-level placement granularity from four reference services to seven native placement units. The measured advantage is concrete: `SFF1` geometry can be placed independently from `Encoder`; `Decoder` can be separated from `User`; `SFF2` / `SFF3` expose route-local completion & backpressure; & the application codecs remain unaware of the chaining protocol. Additional forwarding boundaries & metadata handling are not hidden: they are accounted for by the same route telemetry used in the experimental campaign.

The control placement also changes where work can be shed. In the reference design, temporal selection occurs at the Flow Controller after the Camera has already transmitted the source frame over "TCP"; even a discarded frame must be drained from that incoming stream to preserve framing. Here, `Camera` applies the returned `temporal_skip` **before packetisation**, so an admitted factor `skip = n` reduces subsequent source injection nominally to `TARGET_FPS / n`. The load-shedding decision can therefore suppress raw point traffic at its origin instead of paying the upstream transport / drain cost first. The repeated `B0` / fidelity conditions remain at `skip = 1`; the runtime sweep provides two measured overload examples. The archived legacy `O-OFF` suppresses 11 source frames after duplicated local Encoder recomputation crosses the workload frontier, while `P40` suppresses 30 under point-datagram pressure. The fair `O-OFF` reruns replace the former as the final geometry-placement experiment.

A similar shortening occurs for interactive pose changes. The reference "Pose" command reaches `Encoder`, requiring the newly requested view to traverse projection, encoding, transport, decoding, & reconstruction before it becomes visible. The current "Pose" path terminates at `Decoder`, where the latest stance is applied during reconstruction. This removes re-projection / re-encoding from the **command-to-effect dependency chain** while deliberately leaving the source / encoded representation unchanged. The current NON-QUALITY archive measures this path directly through `reference_cmd_ms`, `cmd_apply_ms`, `Decoder` `pose_control_ms`, & browser `cmd_photon_ms`.

The removal of the former "OVS"-"DPDK" virtual-switch intermediary constitutes a separate topology simplification within the current project lineage. Adjacent microservices communicate directly through project-defined Unix-domain "vhost-user" sockets, eliminating the dedicated virtual-switch "PMD" role & releasing one logical core that had previously been reserved exclusively for switching. On the present 8-logical-"CPU" host, this converts `1 / 8 = 12.5 %` of logical scheduling capacity from switch-only reservation into application-available placement capacity. It is a **resource reallocation**, not evidence that the whole application now consumes 12.5 % less "CPU" time.

At the measurement level, the repository deliberately preserves the experimental principle adopted by the reference pipeline: frame-associated indicators are resolved during execution without synchronous native ".csv" writes inside the real-time path, retained in memory, & serialised only after the terminal condition. The present "SFC" implementation extends that baseline with service-state, protocol-integrity, local "DPDK" backpressure, reverse-control, browser-acknowledgment, & post-stream quality observability while keeping the authoritative application identity anchored to `frame_id`.

Section 21.15 provides the direct numerical comparison supported by the supplied reference material. The measured result is explicit: the best complete runtime trace reaches a lower Camera-to-User frame-ready latency while preserving the same 300-frame / 30 frames / s objective, & the paired offload controls show that relocating geometry in-network reduces `Encoder` workload pressure. Resource-accounting claims such as whole-host "CPU" reduction remain outside the measured output of this campaign, but they do not weaken the demonstrated placement result.

### 1.4 Guidelines & "Codec" Scope

The presentation of the work should make explicit that persistent "FFmpeg" / hardware-"codec" execution is treated as a stateful subsystem, not as a per-frame subprocess. `Encoder` & `Decoder` are therefore stabilised through readiness files, private pre-roll frames, terminal post-roll coordination, & repeated "EOS" propagation. These mechanisms are part of the experimental design because they remove startup / drain ambiguity from the 300 application frames & allow the native telemetry to remain tied to the real service frames only.

Two distinctions remain critical for accurate interpretation of this work.

Firstly, the project integrates the **"SFC" concepts** of "Service Path Identifier" ( "SPI" ) & "Service Index" ( "SI" ), alongside an experimental "MD-Type-2"-like context layout. Nevertheless, the wire representation constitutes a closed project protocol & is appropriately characterised as **"NSH"-inspired**, rather than asserting generic "RFC 8300" interoperability.

Secondly, `Encoder` does **not** implement "MPEG" "V-PCC" or "G-PCC". It constructs a custom 6-view "Geometry" / "Texture" / "Occupancy" "Atlas", employing "HEVC" ( `hevc_nvenc` ) as the video compression engine for this representation. The resulting bitrate & latency metrics characterise the specific projection-&-video path & should not be presented as standards-compliant coding benchmarks.

---

## 🧩 2. Architectural Roles & Why Each Node Matters

| Device | Importance |
|---|---|
| `Camera` | Establishes the absolute source timeline, reads prepared fixed-width frames, serialises coordinates into network byte order, packetises below the "MTU", & applies the most recent `temporal_skip` before frame injection. |
| `SFF1` | Demonstrates the in-path computation principle. It evaluates progressive centroid / extent / raw bounding-box information while forwarding, computes exact frame-completing `max_r`, derives the final projection scale / projected bounding box, & exports the resulting geometry through the experimental service context. |
| `SFF2` | Separates service-path state from unaware application functions. Its four ports implement all three primary-route transitions, plus the reverse "Temporal" & "Pose" paths, while preserving `SPI` / `SI` state around `Encoder` & `Decoder`. |
| `Encoder` | Reconstructs complete frames, consumes `SFF1` geometry when valid, executes the "CUDA" six-view projection, feeds a persistent pre-rolled "NVENC" process, attributes asynchronous "MPEG-TS" output, & regulates source admission from measured workload. |
| `Decoder` | Receives plain `cam_hdr + enc_hdr + MPEG-TS`, feeds a persistent hardware decoder, reconstructs the projected representation into a point cloud, applies the current pose, & emits a packet-sequenced `dec_hdr + point_tx` stream. |
| `SFF3` | Terminates the primary aware chain before the end user & originates the reverse "Pose" service chain. It strips `SPI 100 / SI 253` on the data path & adds `SPI 300 / SI 255` to validated `User` commands. |
| `User` | Validates final point packets, reassembles reconstructed geometries, publishes only completed snapshots to the local Web bridge, tracks pose requests, gathers end-to-end / browser acknowledgments, & coordinates quality capture. |
| "Python" bridge / Viewer | Implements an asynchronous, latest-frame-only presentation frontier. A one-element peer queue & a one-frame-in-flight acknowledgment gate prevent progressive browser backlog from feeding back into the native `User` data path. |
| `Gauge` | Executes only after quality-stream completion, recovering objective geometry metrics without competing with the real-time network path. |

The virtual topology is direct:

```
Camera <-> SFF1 <-> SFF2 <-> Encoder
                    |  |
                    |  +---- Decoder
                    |
                    +------- SFF3 <-> User
```

The exact direct sockets are:

```
Camera <-> SFF1    /tmp/sfc-cam-sff1
SFF1   <-> SFF2    /tmp/sfc-sff1-sff2
SFF2   <-> Encoder /tmp/sfc-sff2-enc
SFF2   <-> Decoder /tmp/sfc-sff2-dec
SFF2   <-> SFF3    /tmp/sfc-sff2-sff3
SFF3   <-> User    /tmp/sfc-sff3-usr
```

There is therefore no additional virtual-switch processing tier. `SFF1`, `SFF2`, & `SFF3` are the sole nodes that interpret service-chain semantics, while `Encoder` & `Decoder` remain intentionally "SFC"-unaware.

---

## 🔗 3. Service-Chain Semantics

### 3.1 "Main" Service Path

The project defines:

```
MAIN_SPI = 100
```

Primary service state evolves as:

```
SFF1 emits aware state                         : "SPI 100", "SI 255" + geometry context
SFF2 captures state & removes service envelope : Encoder receives plain "UDP" + geo_agg_hdr
Encoder returns plain compressed traffic       : proxy state advances "SI 255 -> 254"
SFF2 forwards plain application traffic        : Decoder remains "SFC"-unaware
Decoder returns reconstructed points           : proxy state advances "SI 254 -> 253"
SFF2 re-imposes base service envelope          : "SPI 100", "SI 253"
SFF3 validates & removes base envelope         : User receives plain "UDP" + dec_hdr + points
```

The entire route is operational:

```
Camera
  -> SFF1 ( "GAC", aware boundary )
  -> SFF2 ( Route 0, proxy capture / decapsulation )
  -> Encoder ( unaware )
  -> SFF2 ( Route 1, proxy transition )
  -> Decoder ( unaware )
  -> SFF2 ( Route 2, proxy transition / re-encapsulation )
  -> SFF3 ( aware boundary / decapsulation )
  -> User
```

Route-specific completion, byte accounting, protocol validation, & Tx acceptance telemetry are retained independently by `SFF2` for all three transitions.

### 3.2 "Temporal" Service Path


"Temporal" adaptation utilises:

```
TEMPORAL_SPI = 200
TEMPORAL_SI  = 255
```

`Encoder` generates the request from its internal workload model. The current packed control structure is:

```
frame_id  : uint32
timestamp : uint64
skip      : uint16
padding   : uint16
------------------
Total     : 16 B
```

The path is:

```
Encoder
  -> plain 16-B "UDP" payload
  -> SFF2 classifies + imposes "SPI 200 / SI 255"
  -> SFF1 validates + removes the service envelope
  -> Camera validates the exact 16-B datagram & accepts a non-older frame-associated decision
```

The command carries the **absolute requested factor**, rather than an increment / decrement event. `Camera` normalises an invalid zero factor back to `1` & accepts the request only when its `frame_id` is not older than the most recently accepted control reference. Once adopted, source admission is evaluated before point packetisation according to:

```
selected( frame_id ) = ( ( frame_id - 1 ) mod skip ) == 0
FPS                  = TARGET_FPS / skip
```

Consequently, temporal reduction occurs at the earliest useful frontier: skipped source frames do not consume point packetisation, `Camera` Tx descriptors, or any subsequent primary-route service capacity. For example, if a controlled overload experiment requests `skip = 2`, the nominal admitted source rate becomes `15 frames / s`; `skip = 3` yields `10 frames / s`. These are deterministic consequences of the selection rule, not measured outcomes of the final `skip = 1` archives.

Reliability is expressed as returned-state convergence. If `workload_controller.requested_skip != active_skip`, `Encoder` suppresses further overload / recovery transitions & re-dispatches the desired factor after `RETRY_FRAMES = 3` observations. The request is considered semantically reflected only when subsequent `cam_hdr.temporal_skip` values return the requested state. A lost control datagram can therefore delay adoption; the application continues re-presentation while observations remain available, & convergence is conditional upon at least one subsequent copy being successfully delivered. This mechanism is intentionally distinct from transport retransmission: it verifies the **application effect**, not merely delivery to a socket.

### 3.3 "Pose" Service Path


"Pose" control remains independent from temporal regulation:

```
POSE_SPI = 300
POSE_SI  = 255
```

The packed command is:

```
timestamp : uint64
yaw       : uint32 network-order float bit pattern
pitch     : uint32
zoom      : uint32
padding   : uint32
------------------
Total     : 24 B
```

The operational route is:

```
Browser
  -> Python bridge shared control map
  -> User ( plain "UDP" )
  -> SFF3 ( classify + impose "SPI 300 / SI 255" )
  -> SFF2 ( validate + decapsulate )
  -> Decoder ( plain "UDP" command )
```

The payload represents an **absolute stance**, rather than a relative rotation event. `User` timestamps the first dispatch, stores the requested `yaw`, `pitch`, & `zoom`, & re-presents the same timestamped state every `RETRY_FRAMES = 3` completed frames while it remains unresolved. `Decoder` accepts finite positive-zoom commands, discards an already consumed timestamp, rejects any timestamp older than the active stance, snapshots the latest admissible state before reconstruction, & applies that state on the "GPU". This combination makes retransmitted duplicates idempotent & prevents a delayed older command from rolling the view backward.

The reconstructed frame returns the pose actually applied by `Decoder`. `User` therefore closes the semantic loop by matching the returned `yaw` / `pitch` / `zoom` against the command record, rather than assuming success after local Tx acceptance. A newer browser command replaces the currently active unresolved request, reflecting the latest-state semantics expected from direct manipulation.

The browser-facing command path remains non-blocking at issuance: command submission does not wait synchronously for an acknowledgment before returning control to the interface. Correctness is instead enforced by the native `User` loop: an unresolved "Pose" keeps its original timestamp & is re-presented every `RETRY_FRAMES = 3` completed frames until the returned reconstructed stream carries the requested stance. Browser acknowledgments remain presentation telemetry & one-frame-in-flight Web scheduling signals, not the native reliability condition.

### 3.4 Protocol Clarification

The architecture employs an 8-byte `nsh_hdr`, the "SPI" / "SI" paradigm, a "TTL" field, & a project-defined "MD-Type-2"-like geometric context. The implementation is accurately described as **experimental / "NSH"-inspired**, rather than as a universally interoperable "RFC 8300" implementation. "RFC 7665" is relevant at the architectural level because it defines SFFs, "SFC"-aware / unaware functions, proxies, classification, metadata exchange, & topology-independent service paths; "RFC 8300" motivates the `SPI` / `SI` progression & proxy behaviour. Neither referenced specification requires the project to employ "UDP" as its transport, so transport uniformity is an implementation choice rather than a standards-compliance claim.

#### 3.4.1 Project Labels & "RFC 7665" Role Mapping

The executable names `SFF1`, `SFF2`, & `SFF3` are **project placement labels**, not assertions that each process corresponds one-to-one with a single abstract "RFC 7665" role. The implementation deliberately co-locates compatible forwarding, classification, proxy, & service-function responsibilities inside a constrained software Data Plane:

| Project Node | "RFC 7665"-Oriented Logical Roles | Concrete Project Interpretation |
|---|---|---|
| `SFF1` | Classifier + "SFF" responsibilities + co-located "SFC"-aware geometry "SF" | Classifies the source-side primary path, imposes / validates experimental service state, executes the "GAC" geometric treatment while forwarding, & terminates the reverse "Temporal" aware envelope before `Camera` |
| `SFF2` | "SFF" + "SFC Proxy" responsibilities | Retains / advances service-path state around the intentionally "SFC"-unaware `Encoder` & `Decoder`, removes the aware envelope before those functions, & reconstructs the required state when traffic re-enters the aware chain |
| `SFF3` | Final primary-path "SFF" / declassification boundary + reverse-path Classifier | Validates / removes the final primary service state before `User`, while independently classifying plain 24-byte "Pose" commands & imposing the reverse `SPI 300 / SI 255` service state |

This mapping preserves the architectural vocabulary without pretending that software-process boundaries must mirror the abstract role boundaries literally. In particular, the geometry treatment performed inside `SFF1` is the service-function computation, while the same process also owns forwarding / classification duties required by the experimental placement.

The service-plane constants are fixed as part of the experiment:

```text
MAIN_SPI                   = 100
MAIN_SI_SFF1               = 255
MAIN_SI_ENCODER            = 254
MAIN_SI_DECODER            = 253

TEMPORAL_SPI               = 200
TEMPORAL_SI                = 255
POSE_SPI                   = 300
POSE_SI                    = 255

DEFAULT_TTL                = 63
MD_CLASS_EXPERIMENTAL      = 0xFFF6
MD_TYPE_GEOMETRY           = 0x01
MD_TYPE_2                  = 0x02
NEXT_PROTOCOL_EXPERIMENT_1 = 0xFE
```

The corresponding protocol contract can be summarised without relying upon implicit application behaviour:

| Family | Classification / Entry | Service State | Fixed Application Entity | Terminal Acceptance Rule |
|---|---|---|---|---|
| "Main" | `Camera` point datagrams enter through `SFF1`; subsequent proxy transitions are selected by ingress port & preserved primary state | `SPI 100`, `SI 255 -> 254 -> 253` | `cam_hdr + points`, then `cam_hdr + enc_hdr + MPEG-TS`, then `dec_hdr + points` | Exact frame / packet populations, route-specific metadata, & expected `SPI` / `SI` at aware boundaries |
| "Temporal" | Exact 16-B plain payload from `Encoder`-facing port | `SPI 200 / SI 255` between `SFF2` & `SFF1` | `temporal_payload` | Nonzero normalised absolute skip, non-older frame reference, returned `cam_hdr.temporal_skip` confirmation |
| "Pose" | Exact 24-B plain payload at `SFF3`, then exact base-aware payload at `SFF2` | `SPI 300 / SI 255` between `SFF3` & `SFF2` | `pose_payload` | Finite stance, positive zoom, monotonic timestamp, returned pose matching at `User` |

This explicit contract is central to research validity: route selection, packet shape, accepted state transition, & completion criteria are fixed in code rather than inferred retrospectively from traffic.

The current address contract is:

| Adjacency / Endpoint | "IPv4" Address | "UDP" Port |
|---|---|---:|
| `Camera` | `10.0.1.1` | `5001` |
| `SFF1` `Camera`-facing endpoint | `10.0.1.254` | `6633` |
| `SFF1` `SFF2`-facing endpoint | `10.0.2.1` | `6633` |
| `SFF2` `SFF1`-facing endpoint | `10.0.2.2` | `6633` |
| `Encoder` | `10.0.3.1` | `7001` |
| `SFF2` `Encoder`-facing endpoint | `10.0.3.254` | `6633` |
| `Decoder` | `10.0.4.1` | `8001` |
| `SFF2` `Decoder`-facing endpoint | `10.0.4.254` | `6633` |
| `SFF3` `SFF2`-facing endpoint | `10.0.5.1` | `6633` |
| `SFF2` `SFF3`-facing endpoint | `10.0.5.2` | `6633` |
| `User` | `10.0.6.1` | `9001` |
| `SFF3` `User`-facing endpoint | `10.0.6.254` | `6633` |

These allocations form a closed testbed contract rather than a dynamically routed deployment. Native nodes construct & validate the corresponding "Ethernet" / "IPv4" / "UDP" envelopes directly through the six Unix-domain "DPDK" adjacencies.

When an "IPv4" header is generated or rewritten, its checksum is cleared & recomputed via `rte_ipv4_cksum()`. The project deliberately maintains a zero "UDP" checksum for the present "IPv4" testbed. "RFC 768" permits a transmitted zero checksum to indicate that no checksum was generated, but "RFC 8085" recommends checksum protection for general use; the current choice is therefore a **restricted experimental convention**, compensated by a closed local topology plus explicit application / protocol integrity validation, & must not be generalised to future "IPv6" or unrestricted Internet operation.

---

## 📦 4. Data Representation & Packet Formats

### 4.1 Endianness & Portability

Offline ( `.bin` ) & live network representations fulfil distinct responsibilities. `converter.py` constructs contiguous 16-byte host records using Little-Endian `float32` coordinates; network-facing components reinterpret these floating-point bit patterns as 32-bit words & serialise them in network byte order. All integer protocol fields are likewise explicitly converted at their boundaries.

Consequently, the storage artefact is a documented local representation, whereas the live "DPDK" path is an explicit network representation. `Decoder`, `SFF3`, & `User` preserve the identical coordinate convention when reconstructed points are returned downstream.

### 4.2 Common Structures

| Structure | Size | Function |
|---|---:|---|
| `point_tx` | `16 B` | Network point record containing `x`, `y`, `z`, "RGB", & one padding byte |
| `host_point` | `16 B` | Host / shared-memory point record with native `float` coordinates & "RGB" |
| `cam_hdr` | `40 B` | Frame identity, packet sequence, `Camera` timestamp, source pose, temporal skip, original-point count, & packet population |
| `nsh_hdr` | `8 B` | Experimental service-chain base carrying "SPI", "SI", "TTL", metadata type, & next-protocol information |
| `nsh_md2_ctx_hdr` | `4 B` | Project geometric-context descriptor |
| `geo_agg_hdr` | `64 B` | Centroid, extent, raw bounding-box centre, `max_r`, final scale, global projection scale, projected bounding-box centre, & active-point count |
| `enc_hdr` | `48 B` | Encoded-media packet identity & reconstruction parameters |
| `dec_hdr` | `52 B` | Reconstructed-frame identity, sequence, original / arrived / eroded / valid populations, pose, temporal skip, & packet population |
| `temporal_payload` | `16 B` | Frame / timestamp-associated temporal request |
| `pose_payload` | `24 B` | Timestamped `yaw`, `pitch`, `zoom`, & alignment padding |
| `web_hdr` | `72 B` | Shared-memory snapshot metadata consumed by the asynchronous Web bridge |
| `web_ctrl` | `56 B` | Shared command + browser-acknowledgment exchange structure |
| `quality_hdr` | `8 B` | quality-capture record identity & reconstructed point count |

### 4.3 Point Record — 16 Bytes

```
+--------------------+ 4 B
| x : float32        |
+--------------------+ 4 B
| y : float32        |
+--------------------+ 4 B
| z : float32        |
+--------------------+ 1 B
| r : uint8          |
+--------------------+ 1 B
| g : uint8          |
+--------------------+ 1 B
| b : uint8          |
+--------------------+ 1 B
| padding : uint8    |
+--------------------+

Total = 16 B
```

The deterministic 16-byte width is shared by the offline Converter, live point packets, reconstructed host buffers, quality capture, & browser payload extraction.

### 4.4 Camera Header

The 40-byte `cam_hdr` carries:

```
frame_id
sequence_number
timestamp
yaw
pitch
zoom
temporal_skip
original_points
points_in_packet
padding
```

The `Camera` introduces the neutral source pose ( `0`, `0`, `1` ). `User`-originated pose changes are deliberately applied at `Decoder`, rather than altering the source / `Encoder` projection state. The legacy `padding` field transports the configured point capacity in network byte order. `Camera` writes `POINTS_PER_PACKET`; receivers on the upstream point path use that advertised capacity to derive packet offsets & completion, with zero accepted only as a compatibility fallback to the locally compiled setting. The reconstructed path exposes the corresponding campaign-wide value in `dec_hdr.padding`. `points_in_packet` continues to carry the actual datagram population, so the final packet may contain the natural remainder.

### 4.5 Geometric Metadata Produced by SFF1

The "GAC" exports both packet-progressive raw geometry & frame-complete projection geometry. For the observed prefix `P_N`, the progressive state is:

```
C_N = ( 1 / N ) * sum_{ p in P_N }( p )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

At frame completion, `SFF1` first resolves the exact centroid-dependent radius:

```
max_r = max_{ p in P_frame } || p - C_final ||_2
```

It then materialises the reference-compatible projection frontier inside the offloading node:

```
final_scale  = ( CAMERA_DISTANCE * 0.2 ) / max_r
p'           = ( p - C_final ) * final_scale + ( 0, 0, CAMERA_DISTANCE )
global_scale = max( extent_x' / WIDTH, extent_y' / HEIGHT, extent_z' / WIDTH ) * ( 1 + PADDING )
B'           = ( p'_min + p'_max ) / 2
```

`Encoder` validates the final active-point count before accepting these offloaded values. For complete frames, it consumes `final_scale`, `global_scale`, & projected bounding-box centre directly. When offloading is disabled, the local "CPU" path now reconstructs the **same complete numerical frontier**: sums / extrema, centroid, point-wise `sqrtf()` radius, `final_scale`, point-wise transformed minima / maxima, projected bounding-box centre, & `global_scale` are evaluated in the same order as the `SFF1` formulation before the frame enters the common "CUDA" projection path. Partial / invalid upstream states retain a compatibility fallback, but the nominal `O-OFF` experiment therefore changes the location of computation rather than the geometric definition being evaluated.

### 4.6 Encoder Structure

The 48-byte `enc_hdr` accompanies the compressed stream:

```
frame_id
packet_id
global_scale
box_center_x
box_center_y
box_center_z
yaw
pitch
final_scale
centroid_x
centroid_y
centroid_z
```

`packet_id` resets for each attributed encoded application frame. The remaining parameters allow `Decoder` to reconstruct the six-view projection using the identical geometric convention employed by `Encoder`.

### 4.7 Decoder Output Structure

The 52-byte `dec_hdr` restores frame semantics after decoding / reconstruction:

```
frame_id
sequence_number
timestamp
yaw
pitch
zoom
temporal_skip
padding
original_points
arrived_points
eroded_points
valid_points
points_in_packet
```

`timestamp` retains the original `Camera` timeline. `arrived_points`, `eroded_points`, & `valid_points` distinguish the progressive reconstruction stages, while the pose fields report the stance actually applied by `Decoder`. The legacy `padding` member carries the point-packet capacity selected for the campaign execution. `Decoder` emits the common `POINTS_PER_PACKET` value fixed across the binaries rebuilt for that campaign point, enabling `SFF2`, `SFF3`, & `User` to validate offsets & final remainders without inferring reconstruction population from packet counts alone.

### 4.8 User Shared-Memory & Quality Records

`User` reconstructs complete frames into the shared point region following a 72-byte `web_hdr`. An odd / even sequence protocol provides a lock-free publication marker: odd values indicate an in-progress frame, while an even value identifies a completed stable snapshot. In `QUALITY_CAPTURE = 1`, Web publication & sequence mutation are disabled, as no browser process participates in the experiment.

The 56-byte `web_ctrl` region carries command sequence, command identifier, command type, requested pose, acknowledgment sequence, rendered frame, rendered command, & measured browser `Command-to-Photon` latency.

Quality capture serialises a stream of:

```
quality_hdr { frame_id, point_count } followed by point_count * host_point
```

This representation permits `gauge.py` to index reconstructed frames directly without introducing a second conversion format.

---

## 📐 5. "MTU"-Aware Packet Design

The system targets the conventional:

```
"IPv4" "MTU" = 1500 B
```

All application datagrams remain below that threshold. "Ethernet" framing is reported separately because the "MTU" excludes the 14-byte "Ethernet" header.

Three packetisation parameters are now explicit across the affected native nodes:

```text
POINTS_PER_PACKET  = 80
NETWORK_MTU        = 1500
MEDIA_PAYLOAD_SIZE = 1316
```

`POINTS_PER_PACKET` sets the maximum point population of a datagram rather than requiring every packet to reach that size. `cam_hdr.padding` advertises the configured capacity on the source point path, whereas `dec_hdr.padding` carries the corresponding campaign value after reconstruction. The final datagram retains its actual `points_in_packet` remainder; a zero advertised capacity is interpreted solely as a compatibility fallback to the local build setting. `NETWORK_MTU` controls the maximum accepted "IPv4" datagram, contributes to the native mbuf-data-room calculation, & is checked against the largest constructed packet at startup.

Compressed media is parameterised separately through `MEDIA_PAYLOAD_SIZE`. The current `1316 B` value deliberately equals `7 * 188 B`, but the application parser does not rely upon arbitrary pipe-read boundaries: "MPEG-TS" is reconstructed internally as complete 188-byte units, whereas `SFF2` & `Decoder` concatenate the received media bytes without imposing an unnecessary network-payload modulo-188 rule. This preserves the current efficient seven-packet grouping while keeping transport sizing independent from the internal parser state.

### 5.1 Camera -> SFF1

```
POINTS_PER_PACKET = 80
POINT_SIZE_BYTES  = 16
Point payload     = 1280 B
```

```
"IPv4"                     20 B
"UDP"                       8 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1348 B
"Eth" frame              1362 B
"IP"-"MTU" margin         152 B
```

### 5.2 SFF1 -> SFF2 -> Encoder

Aware `SFF1` output:

```
"IPv4"                     20 B
"UDP"                       8 B
nsh_hdr                     8 B
nsh_md2_ctx_hdr             4 B
geo_agg_hdr                64 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1424 B
```

Route 0 removes the service base / context while preserving `geo_agg_hdr` for the unaware `Encoder`:

```
20 + 8 + 64 + 40 + 1280 = 1412 B
```

### 5.3 Encoder -> SFF2 -> Decoder

```
TS_PACKET_SIZE     = 188 B
MEDIA_PAYLOAD_SIZE = 7 * 188 = 1316 B
```

```
"IPv4"                     20 B
"UDP"                       8 B
cam_hdr                    40 B
enc_hdr                    48 B
7 * "MPEG-TS"            1316 B
-------------------------------
"IPv4" datagram          1432 B
"IP"-"MTU" margin          68 B
```

### 5.4 Decoder -> SFF2 -> SFF3 -> User

`Decoder` emits a plain packet containing a 52-byte reconstruction header:

```
"IPv4"                      20 B
"UDP"                        8 B
dec_hdr                     52 B
80 * point_tx             1280 B
--------------------------------
Decoder -> SFF2           1360 B
```

Route 2 re-imposes the 8-byte service base:

```
SFF2 -> SFF3 = 1360 + 8 = 1368 B
```

`SFF3` removes that base before `User`, returning the datagram to `1360 B`. The complete reconstructed-point route therefore retains at least `132 B` of "IPv4" "MTU" headroom at its largest aware boundary.

### 5.5 "Temporal" & "Pose" Packets


"Temporal":

```
Encoder -> SFF2 plain   = 20 + 8 + 16     = 44 B
SFF2    -> SFF1 aware   = 20 + 8 + 8 + 16 = 52 B
SFF1    -> Camera plain = 44 B
```

"Pose":

```
User -> SFF3 plain    = 20 + 8 + 24     = 52 B
SFF3 -> SFF2 aware    = 20 + 8 + 8 + 24 = 60 B
SFF2 -> Decoder plain = 52 B
```

Every command therefore fits in one unfragmented datagram & preserves a direct one-message / one-packet relation at the application boundary. This property is used by the native classifiers: no command stream has to be reconstructed before the corresponding service path can be selected.

For the exact current payloads, the **minimum-header** comparison against an otherwise identical "IPv4" / "TCP" carrier is:

| Control Entity | Current "UDP" L3 Packet | Hypothetical Minimum-"TCP" L3 Packet | Current Byte Reduction |
|---|---:|---:|---:|
| "Temporal" plain | `20 + 8 + 16 = 44 B` | `20 + 20 + 16 = 56 B` | `21.4 %` |
| "Temporal" aware | `20 + 8 + 8 + 16 = 52 B` | `20 + 20 + 8 + 16 = 64 B` | `18.8 %` |
| "Pose" plain | `20 + 8 + 24 = 52 B` | `20 + 20 + 24 = 64 B` | `18.8 %` |
| "Pose" aware | `20 + 8 + 8 + 24 = 60 B` | `20 + 20 + 8 + 24 = 72 B` | `16.7 %` |

Including the 14-B "Ethernet" header, the corresponding reductions are approximately `17.1 %`, `15.4 %`, `15.4 %`, & `14.0 %`. These percentages are deliberately conservative: they assume the minimum 20-B "TCP" header, no "TCP" options, & no acknowledgment-only traffic. Conversely, they do **not** count a three-way handshake per command, because a sensible comparison would employ a persistent "TCP" control channel as the reference architecture does. The table therefore isolates the transport-envelope difference without constructing an artificially weak "TCP" baseline.

At the measured command rates, the 12-byte minimum transport-header difference is not the dominant system cost. The practical reason for retaining "UDP" is that every current control remains one fixed-width, self-delimiting datagram which `SFF2` / `SFF3` classify directly, while duplicate / loss handling is verified against the returned application state rather than hidden behind byte-stream delivery.

The two command families are structurally independent & carry their own timing / state information. "Temporal" uses frame-associated state confirmation; "Pose" preserves one command timestamp across retries & relies upon `Decoder` monotonicity. Consequently, command issue remains asynchronous & non-blocking, while control correctness is tied to returned application state rather than to a hidden transport acknowledgment.

### 5.6 "Virtio" Queue Size & "DPDK" Rx / Tx Dimensioning

The native configuration normalises every `rte_eth_rx_queue_setup()` & `rte_eth_tx_queue_setup()` call to:

```
4096 descriptors
```

The corresponding `User` `virtio-user` attachment also employs:

```
queue_size = 4096
```

This final value is a measured experimental condition rather than an assertion that larger queues intrinsically improve performance. Descriptor depth, `virtqueue` capacity, core affinity, bounded zero-accept retries, & consumer service time form one joint buffering / scheduling condition. Prior queue-depth experiments are therefore not silently mixed with the final result set.

Each native node retains:

```
NUM_MBUFS        = 16383
MBUF_CACHE_SIZE  = 256
BURST_SIZE       = 32
MAX_ZERO_ACCEPTS = 2048
```

A zero return from `rte_eth_tx_burst()` represents **local Tx-ring acceptance pressure**, not a network-level "UDP" retransmission. Native telemetry consequently records zero accepts, partial accepts, resubmission calls, & re-presented packet counts separately from frame-completion semantics.

`MAX_ZERO_ACCEPTS` bounds the **consecutive** zero-accept streak associated with a pending burst rather than the cumulative `tx_zero_accepts` value of an entire frame. Any positive acceptance resets the local retry streak; consequently, a frame may legitimately report more than `2048` total zero-accept events while still reaching `tx_complete = 1`. This distinction is essential when interpreting the comparatively large `Camera` counters reported by the final archives.

---

## 📷 6. Camera — Scheduling, Warm-Mode, "Temporal" Selection, & Telemetry

### 6.1 Role

The `Camera` functions as the exclusive node originating the volumetric schedule. It ingests the pre-converted "Loot" sequence, allocates source frame IDs, serialises coordinates for network transmission, packetises each point cloud, timestamps the element, & submits "DPDK" bursts to `SFF1`. Consequently, its contribution is simultaneously functional & experimental. Every downstream timing quantity is ultimately conditioned by the source cadence, burst structure, cache mode, & residency policy established at this component.

The reference workload is configured as:

```text
K_FRAMES   = 300
TARGET_FPS = 30.0
BURST_SIZE = 32
```

### 6.2 Input Modes

Three distinct cache / storage modes are implemented:

| **Mode** | **Allocation** | **fread()** | **Meaning** |
| -------------------------------- | ----------------- | ----------------- | ----------------------------------------------------------------------------------------------- |
| `CACHE_MODE_BEST`                | Before streaming  | Before streaming  | Cleanest datapath-oriented experiment; frame bytes already reside within the application buffer |
| `CACHE_MODE_MIDDLE`              | Before streaming  | Inside frame loop | Storage-aware experiment; disk / page-cache access remains visible within `Camera` residency      |
| `CACHE_MODE_WORST`               | Inside frame loop | Inside frame loop | Deliberately pessimistic mode incorporating allocation & file read within the frame path        |

The source configuration selects:

```text
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

`WARM_MODE_ENABLED` maps & locks source documents prior to the measured sequence, ensuring the timed file-acquisition path retains standard buffered-read semantics while operating across a resident file-backed working set. The resulting `disk_io_ms` should be interpreted as **timed buffered acquisition from a warmed condition**, rather than a direct measurement of cold physical-storage latency. In `CACHE_MODE_MIDDLE` the interval covers `fopen()` / `fread()` / `fclose()` into the reusable staging buffer; in `CACHE_MODE_WORST` the same timer additionally begins before the per-frame allocation, while `CACHE_MODE_BEST` reports zero because acquisition occurs before streaming.

The completed source-residency sweep confirms that this distinction is operational rather than nominal. Both `MIDDLE` & `WORST` sustain approximately `30 frames / s` when the file-backed working set is resident, whereas disabling warm mode in either case raises the timed `fread()` median to approximately `115 ms` & reduces the observed source cadence to about `7.88 frames / s`. `BEST` & `WORST` consequently serve as useful sensitivity bounds rather than the principal operating condition: `BEST` shifts acquisition / serialisation outside the timed loop, while `WORST` adds per-frame allocation / release that is not required by the reference-like source semantics. `MIDDLE + WARM` retains the per-frame read boundary while controlling page residency, making it the most suitable runtime comparator for the nominal 30-fps campaign.

Results obtained under varying settings constitute distinct experimental conditions & must not be conflated within identical performance claims.

### 6.3 Packetisation

Each selected frame is partitioned into packets containing a maximum of 80 points. The `Camera` assigns:

```text
frame_id
sequence_number
timestamp = t_camera_send
temporal_skip
original_points
points_in_packet
```

The propagated `Camera` timestamp is `t_camera_send`, whereas `tx_start_timestamp` records the later `t_send_start` boundary. In `CACHE_MODE_MIDDLE` / `CACHE_MODE_WORST`, `t_camera_send` is sampled after the timed source-acquisition interval & immediately before in-place coordinate serialisation; in `CACHE_MODE_BEST`, it is sampled after selecting the already resident, pre-serialised frame buffer. `t_send_start` is sampled subsequently, immediately before packet submission begins. Consequently, the downstream `Camera`-origin latency frontiers include serialisation & Tx for the principal `MIDDLE + WARM` condition but exclude its preceding file-acquisition interval; `Camera.total_residency_ms` retains that earlier source work. The `cam_hdr.timestamp` value then propagates unchanged across all packets corresponding to the same frame. `cam_hdr.padding` carries the active `POINTS_PER_PACKET` capacity, while `points_in_packet` remains the exact population of the current datagram; sequence offsets are therefore derived from the advertised capacity rather than from a hard-coded 80-point assumption.

Coordinates transition from the prepared little-endian host representation to network-order "IEEE-754" bit patterns prior to transmission. "RGB" values & the explicit padding byte remain byte-valued fields.

### 6.4 Isochronous Scheduling & Camera-Side "Temporal" Selection

The source establishes a singular absolute session origin & computes the ideal deadline for frame index `k` as:

```text
T_ideal( k ) = T_0 + k * ( 1 / TARGET_FPS )
```

For the current configuration:

```text
TARGET_FPS = 30
T_base     = 33.333... ms
```

Prior to each source-frame decision, the `Camera` polls for a `temporal_payload`. Should the current factor be `skip`, the frame is selected when:

```text
( frame_id - 1 ) mod skip = 0
```

& the nominal admitted rate resolves to:

```text
FPS = TARGET_FPS / skip
```

Both selected & skipped frames advance against the identical absolute source timeline. This mechanism prevents temporal adaptation from redefining the session clock, thereby preserving meaningful frame-ID-based scheduling & downstream jitter calculations.

Consequently, the current design omits a separate `PACING_MODE`. Source timing is governed entirely by the absolute target schedule, whereas local Tx-ring pressure is exposed via retry telemetry rather than being obscured behind a supplementary pacing heuristic.

### 6.5 Meaning of Tx Backpressure Counters

A zero return from `rte_eth_tx_burst()` indicates the local Tx path accepted zero packets during that specific attempt. The `Camera` retries employing a bounded pause strategy & records:

```text
tx_zero_accepts
tx_partial_accepts
tx_resubmit_calls
tx_resubmitted_packets
mbuf_starvation
```

These quantities must not be classified as "UDP" retransmissions. They transpire **prior to successful local "DPDK" queue acceptance** & thus measure producer / consumer pressure directly at the local packet-I/O boundary.

The validated run exhibits numerous zero-accept attempts due to the emission of approximately ten thousand point packets per frame; nevertheless, every frame remains complete & `mbuf_starvation = 0`. The counters thereby provide robust evidence that backpressure existed without translating into application-visible loss during this experiment.

### 6.6 Camera Telemetry — Complete Semantics

The final `Camera` exports **31 fields**. The schema distinguishes source selection, control application, packet population, logical / network rates, file acquisition, serialization, active Tx execution, full source residence, & local queue-acceptance pressure:

```text
frame_id;selected;tx_complete;current_skip;last_control_frame;temporal_control_ms;camera_send_timestamp;tx_start_timestamp;inter_departure_ms;tx_points;tx_packets;payload_bytes;reference_size_bytes;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;disk_io_ms;serialization_ms;tx_duration_ms;active_tx_ms;active_process_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id` | Original source identifier from the 300-frame sequence. |
| `selected` | `1` when the frame survives the active temporal admission rule. The repeated `B0` / fidelity conditions retain all 300 frames at `current_skip = 1`; a stress variant may legitimately contain unselected source IDs. |
| `tx_complete` | `1` only when all selected point packets are successfully accepted by the local "DPDK" Tx path. |
| `current_skip` | "Temporal" factor applied when deciding whether this source frame is admitted. |
| `last_control_frame`, `temporal_control_ms` | Last accepted control reference & measured local control-handling delay. |
| `camera_send_timestamp` | Propagated source anchor. For `MIDDLE` / `WORST`, it is sampled after the timed acquisition interval & before coordinate serialisation; for `BEST`, it follows selection of the pre-serialised resident buffer. |
| `tx_start_timestamp` | Later local boundary sampled immediately before packet submission for the selected frame. |
| `inter_departure_ms` | Real elapsed interval between the send-start timestamps of two successive **actually transmitted** frames. The first measured shot reports `0`; frame-ID gaps are not divided away, so temporal selection remains visible in the raw departure interval. |
| `tx_points`, `tx_packets`, `payload_bytes` | Successfully accepted point population, datagram count, & point bytes. |
| `reference_size_bytes` | Logical frame size used by the reference-throughput calculations. |
| `internal_throughput_mbs`, `reference_throughput_mbs` | Local logical-source rates under the measured / reference boundaries, both expressed in decimal MB / s. |
| `logical_bitrate_mbps`, `network_bitrate_mbps`, `reference_bitrate_mbps` | Application, protocol-inclusive, & reference bitrate formulations at the current effective frame rate. |
| `disk_io_ms` | Timed non-`BEST` source-acquisition interval. `MIDDLE` covers `fopen()` / `fread()` / `fclose()` into the reusable staging buffer; `WORST` also includes the preceding per-frame allocation because the timer begins before `malloc()`. |
| `serialization_ms` | In-place conversion of pre-loaded `XYZ` coordinate bit patterns to network byte order. It is zero in `BEST`, where the application-resident buffers are serialised before streaming. |
| `tx_duration_ms` | Wall-clock span across frame Tx submission, including local retry waits. |
| `active_tx_ms` | Time actively spent inside `rte_eth_tx_burst()` calls, excluding deliberate pause / retry gaps. |
| `active_process_ms` | Project-defined source processing aggregate `disk_io_ms + serialization_ms + tx_duration_ms`. Despite the historical field name, it includes Tx wall-clock retry waiting & is not a pure "CPU"-active interval. |
| `total_residency_ms` | Full selected-frame `Camera` residence from local processing start — before non-`BEST` acquisition — to accepted terminal packet. |
| `node_efficiency_pct`, `reference_efficiency_pct` | Ratios against `total_residency_ms`; the measured numerator includes acquisition + serialisation + Tx duration, while the reference numerator omits serialisation. They are project-defined efficiency indicators rather than host-"CPU" utilisation. |
| `tx_zero_accepts` | Number of local Tx calls accepting no packet. |
| `tx_partial_accepts` | Number of local Tx calls accepting fewer packets than requested. |
| `tx_resubmit_calls`, `tx_resubmitted_packets` | Local re-presentation operations & requested packet-attempt population. These are not "UDP" retransmissions. |
| `mbuf_starvation` | Packet-buffer allocation failures. The retained validation executions record `0`. |

The distinction between `tx_duration_ms` & `active_tx_ms` remains essential. A frame can spend additional wall-clock time waiting for local descriptor availability without the "CPU" simultaneously executing inside `rte_eth_tx_burst()`. For this reason, `Camera.active_process_ms` must be interpreted according to the explicit aggregate above rather than assumed to share the active-only semantics of every downstream node.

### 6.7 "End-of-Stream" Behaviour

Following the configured sequence, the `Camera` emits recurrent `END_OF_STREAM` control packets containing the `Camera` header & devoid of point payload. This redundancy ensures the terminal condition remains resilient to transient local queue behaviour within the experimental environment.

The "EOS" marker constitutes a protocol event, rather than a supplementary source frame, & is excluded from the 300-frame telemetry table.

---

## 🧠 7. SFF1 — "Geometry-Aware Classifier" ( "GAC" ) & In-Path Aggregation

### 7.1 Role

`SFF1` operates as the "Geometry-Aware Classifier" ( "GAC" ) within the current architecture. Its fundamental purpose is to demonstrate that actionable frame geometry can be derived **while point packets actively traverse the service path**, rather than reconstructing identical statistics initially inside the `Encoder`.

For each valid `Camera` packet, it:

```text
validates "Ethernet" / "IPv4" / "UDP" & Camera metadata
decodes point coordinates from network byte order
updates frame-progressive geometric state
stores "XYZ" within a preallocated frame-local workspace
computes progressive centroid / extent / bounding-box centre
computes exact max_r upon frame completion
replaces the outer network envelope in place
appends experimental "NSH" + geometric context
forwards the original Camera application payload
records frame-level telemetry
```

The "GAC" is purposefully designed to exceed the capabilities of a mere forwarding label while remaining substantially narrower than a full-fledged application processor.

### 7.2 "Temporal" Control Is Relayed, Not Decided, by SFF1

The current `SFF1` iteration does **not** execute source-frame temporal filtering.

"Temporal" adaptation is orchestrated by the `Encoder` & enacted by the `Camera`. `SFF1` functions solely as the terminal service-chain-aware relay along the reverse "Temporal" trajectory:

```text
SFF2 -> SFF1   : "NSH"-encapsulated temporal_payload
SFF1           : validates "SPI" = 200 / "SI" = 255
SFF1           : strips the service-chain envelope
SFF1 -> Camera : plain "UDP" temporal_payload
```

The `current_skip` observed by `SFF1` on the primary data path is, therefore, the factor already adopted by the `Camera` & encapsulated within `cam_hdr`. It serves telemetry & effective-rate interpretation purposes, rather than acting as a secondary local frame-drop determinant.

### 7.3 Progressive Geometric Offloading

For an active prefix containing `N` points, `SFF1` maintains:

```text
S_N     = sum_i( p_i )
p_min,N = component-wise min_i( p_i )
p_max,N = component-wise max_i( p_i )
```

& incrementally derives:

```text
C_N = S_N / N
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

These operations permit updating upon each point's arrival, naturally aligning with the computational profile of a data-plane-oriented classifier.

The exact farthest-point radius presents a distinct mathematical dependency:

```text
max_r = max_i || p_i - C_final ||_2
```

Since `C_final` remains indeterminate until the final point is ascertained, exact `max_r` cannot be resolved universally from the initial packet without revisiting preceding points. Consequently, the current implementation records only `XYZ` coordinates within a preallocated workspace during forwarding & executes the exact radius pass upon frame completion.

A second frame-complete pass then materialises the reference-compatible projection frontier inside `SFF1`:

```text
p'_i         = ( p_i - C_final ) * final_scale + ( 0, 0, CAMERA_DISTANCE )
projected B = ( min_i( p'_i ) + max_i( p'_i ) ) / 2
global_scale from transformed extents
```

This pass is intentionally **not** executed for every intermediate packet. Progressive sums / extrema remain inexpensive during traversal, whereas centroid-dependent radius & transformed projection frontiers stay behind the mathematically necessary completion barrier. The resulting design keeps the offloading requirement intact: complete frames arrive at `Encoder` with final projection metadata already attached, while the local complete-frame fallback reproduces the same point-wise radius & transformed-frontier sequence when that metadata is intentionally ignored or unavailable.

#### 7.3.1 Compile-Time In-Network Processing Ablation

`SFF1` now exposes the same style of compile-time selection already used by `Encoder`:

```c
#define NETWORK_PROCESSING_DISABLED 0
#define NETWORK_PROCESSING_ENABLED 1

#define NETWORK_PROCESSING NETWORK_PROCESSING_ENABLED
```

`NETWORK_PROCESSING_ENABLED` remains the default validated condition. When the flag is disabled, `SFF1` preserves classification, packet validation, "NSH" insertion, forwarding, reverse "Temporal" decapsulation, Tx accounting, & all non-geometric telemetry, while **skipping both the packet-progressive point scans & the frame-final `max_r` / transformed-frontier passes**. The geometric context remains structurally present but carries zero / unavailable geometry, with `active_point_count = 0`; no frame-local geometry workspace is allocated or first-touched in that build.

The fair runtime `O-OFF` ablation therefore uses the paired configuration:

```text
SFF1    : NETWORK_PROCESSING = NETWORK_PROCESSING_DISABLED
Encoder : OFFLOAD_MODE       = OFFLOAD_MODE_DISABLED
```

Because disabled `SFF1` intentionally exports `active_point_count = 0` as an unavailable-geometry sentinel, `Encoder` keeps its strict metadata validation only when offload consumption is enabled:

```cpp
if ( unlikely( incoming_active_points > original_points || ( OFFLOAD_MODE && incoming_active_points < points_in_packet ) ) ) {
  rte_pktmbuf_free( m );
  continue;
}
```

`OFFLOAD_MODE` is a compile-time constant, so the fair `O-OFF` build accepts the structurally present zero geometry context without adding a runtime fallback probe, while the baseline build retains the original strict validation.

All remaining packetisation, "DPDK" forwarding, "SFC" steering, source configuration, persistent "codec" state, & workload-control settings stay unchanged. This isolates **where the geometric computation is executed**: in the baseline, progressive / final geometry is resolved in-path by `SFF1`; in fair `O-OFF`, the same frontier is reconstructed only once, locally by `Encoder`. The earlier `O-OFF` archive in which `SFF1` still computed metadata while `Encoder` deliberately ignored it is retained solely as a legacy duplicated-work diagnostic & must be superseded by the new run for the final placement attribution.

### 7.4 In-Place Header Replacement

The incoming `Camera` datagram encompasses:

```text
[ "Ethernet" | "IPv4" | "UDP" | cam_hdr | points ]
```

`SFF1` strips the original network envelope & prepends:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr ]
```

while preserving:

```text
[ cam_hdr | points ]
```

The resultant packet is structured as:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr | cam_hdr | points ]
```

This constitutes the central in-path transformation: service metadata is appended directly to the packet during forwarding, eschewing the creation of a disparate frame-level side channel.

### 7.5 Frame-Global Boundary Constraint

Two distinct definitions of a geometric "boundary" exist within the current implementation.

Axis-aligned extrema are packet-progressive:

```text
min_x / max_x
min_y / max_y
min_z / max_z
```

& consequently attain increasing accuracy as packets arrive.

Conversely, a centroid-dependent radial boundary cannot achieve finality until the centroid itself is finalised. Similarly, establishing a boundary contingent upon an arbitrary future user pose necessitates the prior availability of said pose. No packet format inherently resolves this mathematical dependency.

Therefore, the project deliberately avoids artificial preprocessing solutions at the `Camera` or via an offline converter. Such solutions could transmit final geometry prematurely but would negate the fundamental research objective of evaluating in-place execution at the service function **on the data path**. The current design upholds this objective, rendering the unavoidable frame-global step explicit.

### 7.6 SFF1 Telemetry — Complete Semantics

The final `SFF1` exports **43 fields**, combining frame-integrity, in-path geometry, timing, protocol validation, & Tx-backpressure observability:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;geometry_aggregation_ms;max_r_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id`, `current_skip` | Original `Camera` identity & reflected temporal factor. |
| `rx_complete`, `tx_complete` | Exact receive / forward completion predicates derived from frame sequence & point population. |
| `camera_send_timestamp`, `recv_start_timestamp`, `node_exit_timestamp` | Source, first-arrival, & final-forwarding timing anchors. |
| `original_points`, `rx_points`, `tx_points` | Declared, received unique, & successfully forwarded point populations. |
| `rx_packets`, `tx_packets` | Valid received / accepted packet counts. |
| `payload_bytes`, `reference_size_bytes` | Point payload & comparison baseline sizes. |
| `data_integrity_pct` | `100 * rx_points / original_points` for a populated frame. |
| throughput / bitrate fields | `internal_throughput_mbs` & `reference_throughput_mbs` divide decimal application bytes by the real first-to-last receive span; `logical_bitrate_mbps`, `network_bitrate_mbps`, & `reference_bitrate_mbps` remain rate formulations expressed in Mbit / s. |
| `geometry_aggregation_ms` | Accumulated progressive sum / extrema / centroid / extent / raw bounding-box work, plus frame-complete projection-frontier materialisation. |
| `max_r_ms` | Exact frame-completing radius evaluation after the final centroid becomes available. |
| `tx_duration_ms`, `active_tx_ms` | Wall-clock egress span versus active Tx-call execution. |
| `active_process_ms` | Geometry, exact-radius, projection-frontier materialisation, & active-forwarding work. |
| `cycle_ms`, `header_wait_ms` | Per-frame service-loop / initial-header waiting observability. |
| `total_residency_ms` | First valid input packet to accepted final output packet. |
| `node_efficiency_pct` | Active-work ratio against complete node residence, computed as `100 * active_process_ms / total_residency_ms`. |
| `cycle_occupancy_pct` | Residence-to-cycle ratio, computed as `100 * total_residency_ms / cycle_ms`; it quantifies how much of the observed inter-frame interval is occupied by the frame's node residence. |
| `camera_node_ms` | `Camera` source timestamp to first `SFF1` arrival. |
| `schedule_delay_ms`, `inter_arrival_ms`, `instant_jitter_ms`, `desynced_jitter_ms` | Cumulative schedule drift, raw first-packet-to-first-packet shot interval, frame-ID-gap-aware instantaneous deviation, & its smoothed chronology. `inter_arrival_ms` is `0` for the first observed frame. |
| `eth_errors`, `ipv4_errors`, `udp_errors`, `nsh_errors` | Protocol-validation failures. All remain `0` in the retained validation executions. |
| Tx acceptance counters | Local zero / partial accepts & resubmission activity, independent from "UDP" semantics. |

`data_integrity_pct = 100 %` across every 300-frame `B0`, `BQ`, & `OQ-OFF` complete reference / fidelity execution. The geometry telemetry exposes both placement states directly: `B0` / `BQ` carry the active in-network frontier, whereas fair `OQ-OFF` records the disabled `SFF1` geometry state & the corresponding local `Encoder` recomputation. Sensitivity variants whose source admission or terminal completeness changes are scoped separately in Section 21.16.

---

## 🔀 8. SFF2 — Multi-Port "NSH" Proxy & Service-Path Steering

### 8.1 Role & Ports

`SFF2` is the central four-port service-path proxy. Its interfaces are:

```
PORT_SFF1    = 0
PORT_ENCODER = 1
PORT_DECODER = 2
PORT_SFF3    = 3
```

The node does not perform projection, decoding, or rendering. Its responsibility is to validate packet envelopes, preserve service state across unaware functions, classify reverse controls, rewrite the local network envelope, & maintain exact route-specific accounting.

### 8.2 Implemented Primary Routing

Three primary route identifiers are active:

```
ROUTE_SFF1_ENCODER    = 0
ROUTE_ENCODER_DECODER = 1
ROUTE_DECODER_SFF3    = 2
```

Their semantics are:

```
Route 0 : SFF1    -> SFF2 -> Encoder
Route 1 : Encoder -> SFF2 -> Decoder
Route 2 : Decoder -> SFF2 -> SFF3
```

Route 0 receives `SPI 100 / SI 255` with the geometric context, captures proxy state, removes service metadata, & preserves `geo_agg_hdr` as application-visible information. Route 1 receives plain encoded media, validates the retained context, advances `SI 255 -> 254`, & forwards a plain packet to `Decoder`. Route 2 receives a plain reconstructed-point packet, advances `SI 254 -> 253`, rebuilds the base service envelope, & forwards `SPI 100 / SI 253` to `SFF3`.

### 8.3 "Temporal" & "Pose" Control Paths

`SFF2` also implements the two reverse chains:

```
"Temporal" : Encoder plain -> SFF2 -> "SPI 200 / SI 255" -> SFF1
"Pose"     : SFF3 "SPI 300 / SI 255" -> SFF2 -> plain -> Decoder
```

The `Temporal` classifier accepts exactly the packed 16-byte payload from the `Encoder`-facing port. The `Pose` classifier accepts exactly one base service header plus the packed 24-byte command from the `SFF3`-facing port. These controls do not share primary-frame state.

### 8.4 Burst Ownership

Packets are forwarded through bounded bursts while preserving explicit ownership. Accepted packets are removed from the local burst, non-accepted packets remain eligible for local resubmission, & abandoned entries are explicitly released when the bounded retry policy expires. Nominal complete executions do not expose primary-route acceptance pressure inside `SFF2`; active `P40` is the measured exception at Route 0, where `18,521` zero-accept events occur across three frames. Bounded local resubmission absorbs that pressure with `0` partial accepts & preserves all `270` admitted frames through Route 2, so `SFF2` is not the application-visible loss boundary.

### 8.5 Proxy State & Unaware Service Functions

The proxy model remains central to the thesis methodology. `Encoder` & `Decoder` parse no `nsh_hdr`; instead, `SFF2` retains the `SPI` / `SI` transition around them. This makes the service chain observable without contaminating application parsers with service-routing responsibilities.

The relevant primary sequence is:

```
255 -> Encoder -> 254 -> Decoder -> 253 -> SFF3
```

`SFF3` subsequently consumes the final aware state & returns the application packet to a plain end-device form.

### 8.6 Route-Specific Payload Semantics

Route 0 treats point-cloud bytes as `geo_agg_hdr + cam_hdr + point_tx[]`; Route 1 treats application bytes as `cam_hdr + enc_hdr + MPEG-TS`; Route 2 treats them as `dec_hdr + point_tx[]`. This distinction is reflected directly in the `rx_points`, `tx_points`, `rx_media_bytes`, `tx_media_bytes`, `payload_bytes`, reference-size, & integrity calculations.

All three route logs contain exactly 300 application rows in every complete `B0`, `BQ`, `OQ-OFF`, & fair `O-OFF-R1 / R2` execution. Active `P40` instead follows its admitted `270`-frame population, while the archived legacy duplicated-work `O-OFF` follows its earlier admitted population as documented separately in Section 21.16. The completed fair placement reruns therefore preserve the same 300-frame route-completeness contract while relocating the geometric frontier.

### 8.7 SFF2 Telemetry — Complete Semantics

Each route exports the same 43-column schema, allowing cross-route analysis while retaining route-specific point / media interpretations:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_media_bytes;tx_media_bytes;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

`cycle_occupancy_pct` is intentionally distinct from `node_efficiency_pct`: the former expresses `100 * total_residency_ms / cycle_ms`, while the latter expresses `100 * active_process_ms / total_residency_ms`. Across all three routes, `inter_arrival_ms` measures the raw interval between first valid packets of successive observed application frames. Both measured & reference throughputs use the corresponding frame receive span as denominator; the reference quantity is retained in decimal MB / s rather than being conflated with `reference_bitrate_mbps`.

The resulting files are:

```
/shared/log/sff2/telemetry_sff1_enc.csv
/shared/log/sff2/telemetry_enc_dec.csv
/shared/log/sff2/telemetry_dec_sff3.csv
```

---

## ⚙️ 9. Encoder — Hybrid Frame Processing, "CUDA", "Temporal" Control, & "H.265"

### 9.1 Role

The `Encoder` is deliberately engineered to remain "NSH"-unaware. It processes standard "UDP" packets where the application payload initiates with the geometric context imparted by the `SFF2` proxy:

```text
[ geo_agg_hdr | cam_hdr | points ]
```

Its functional paradigm integrates two distinct execution models.

Upon packet ingress, it functions in a data-plane-oriented capacity: coordinates are decoded & written instantaneously into deterministic frame offsets, packet-completion status is incremented continuously, & the optimal geometry snapshot is preserved. Simultaneously, during intensive "CPU" / "GPU" cycles, the implementation sustains network reception through cooperative polling.

Conversely, at the projection phase, a stringent frame-level readiness constraint is enforced, given that the exact active point set & definitive geometry must be fully resolved. Consequently, the `Encoder` constitutes a **hybrid frame-aware entity**, uniquely positioned between a traditional passive application awaiting complete objects & an atomic packet-local data-plane operation.

### 9.2 Frame Assembly

The frame buffer orchestrates tracking for:

```text
original_points
expected_packets
received_points
rx_packets
payload_bytes
packet_received bitmap
first_arrival
last_arrival
frame_ready
cam_hdr snapshot
geo_agg_hdr snapshot
```

Packet sequence numbers definitively govern destination offsets. Therefore, point conversion executes progressively as packets arrive, negating the necessity for an independent post-reception frame conversion traversal.

A standard frame is deemed processable exclusively when all anticipated packets & points are successfully aggregated. Upon an "EOS" condition, a fragmentary final frame may undergo compaction & processing; however, the validated 300-frame test suite comprises only complete frames.

### 9.3 Selecting the Most Complete Geometry Snapshot

`SFF1` continuously broadcasts progressive geometry. Consequently, the `Encoder` isolates the snapshot possessing the maximum valid `active_point_count` identified for that specific frame.

For a finalised frame, `geometry_from_sff1()` authorises the offloaded metrics strictly when:

```text
metadata_active_points == active_point_count
```

& all decoded geometry components demonstrate finiteness & internal consistency.

This mechanism precludes an early progressive packet from being erroneously interpreted as the definitive geometric description.

### 9.4 Frame Readiness & Incomplete Frames

The conventional processing trajectory demands absolute point completeness preceding projection, as the six-view representation & "codec" input inherently rely on frame-level aggregates.

However, this barrier does not imply that antecedent tasks are postponed. Prior to achieving readiness, the `Encoder` has proactively:

```text
validated packet headers
converted point coordinates
placed points at deterministic offsets
updated receive counters
tracked geometric metadata progression
recorded arrival timing
```

The residual barrier is strictly confined to operations yielding inherently frame-level results, avoiding passive stagnation across the computational pipeline.

### 9.5 Division of Geometric Work & Selectable Offload

The build exposes the following configurations:

```text
OFFLOAD_MODE_DISABLED = 0
OFFLOAD_MODE_ENABLED  = 1
OFFLOAD_MODE          = OFFLOAD_MODE_ENABLED ( actual run )
```

When offload is activated & complete "GAC" metadata is validated, the `Encoder` secures:

```text
centroid
extent
raw bounding-box centre
max_r
final_scale
global_scale
projected bounding-box centre
```

without executing redundant local frame scans. Under conditions of complete offloaded frames, telemetry dictates:

```text
geometry_aggregation_ms = 0
max_r_ms                = 0
```

The complete local path is intentionally defined as the computational counterpart of the `SFF1` frontier rather than as an algebraic shortcut. With `OFFLOAD_MODE_DISABLED`, `compute_geometry_locally()` performs the same three logical stages over the assembled point set: ( i ) double-precision coordinate accumulation plus raw extrema, ( ii ) point-wise `sqrtf()` evaluation against the resolved centroid for `max_r`, & ( iii ) point-wise transformation with the resulting `final_scale` to derive transformed extrema, projected bounding-box centre, & `global_scale`. The resulting structure sets `projection_geometry_ready = true`, so complete locally resolved frames enter `run_projection_pipeline()` through the same metadata-consuming branch used by complete offloaded frames.

This alignment is methodologically important. `Encoder.OFFLOAD_MODE` controls whether the downstream application consumes a valid `SFF1` frontier or reconstructs it locally. For the **fair runtime `O-OFF` placement ablation**, that local fallback is paired with `SFF1.NETWORK_PROCESSING_DISABLED`: `SFF1` retains the identical forwarding / classification / service-envelope role but emits an unavailable zero geometry context, so the same frontier is produced exactly once, at `Encoder`. The archived runtime in which `SFF1` still computed the result while `Encoder` repeated it is retained only as a legacy duplicated-work diagnostic.

A partial frame whose progressive `SFF1` geometry remains population-consistent can still reuse the supplied centroid / extent / raw bounding-box centre & recompute the exact radius over the compacted active set. Such non-nominal states leave the complete projection frontier unresolved until the projection helper applies its compatibility path. Baseline `B0` frames are complete before projection, & both completed fair `O-OFF` reruns preserve that same completeness contract while relocating the geometry frontier to `Encoder`.

For both complete paths, the object scale is resolved from:

```text
target_radius = CAMERA_DISTANCE * 0.2
final_scale   = target_radius / max_r
```

followed by the identical point-wise transformed-frontier reduction. The final runtime ablation therefore compares the paired build states `SFF1.NETWORK_PROCESSING_ENABLED + Encoder.OFFLOAD_MODE_ENABLED` versus `SFF1.NETWORK_PROCESSING_DISABLED + Encoder.OFFLOAD_MODE_DISABLED`, without changing the packet-processing graph or the projection geometry definition.

### 9.6 Workload-Driven "Temporal" Controller

"Temporal" adaptation remains centralised at `Encoder`. For each eligible service sample `T_n`, the controller maintains:

```text
T_base           = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n              = EWMA_ALPHA * T_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio   = E_n / T_budget( active_skip )
```

Current parameters are:

```text
TARGET_FPS        = 30
EWMA_ALPHA        = 0.25
MAX_SKIP          = 9
MIN_FRAMES        = 3
STABLE_STREAK     = 3
MAX_FRAMES        = 15
OVERLOAD_STREAK   = 2
RECOVERY_STREAK   = 9
RETRY_FRAMES      = 3
OVERLOAD_RATIO    = 0.90
RECOVERY_RATIO    = 0.75
OVERLOAD_FRACTION = 0.25
RECOVERY_FRACTION = 0.10
```

After the warm-up / stability gate, overload can be asserted when any of the following holds:

```text
workload_ratio >= OVERLOAD_RATIO
raw_queue_ms   > active_budget_ms * OVERLOAD_FRACTION
frame_backlog  >= 2
"codec" backlog is observed to be growing
```

The recovery predicate is intentionally stricter: the projected ratio at `skip - 1` must not exceed `RECOVERY_RATIO`, `raw_queue_ms` must remain within the recovery fraction, `frame_backlog` must be zero, & "codec" backlog must not be increasing. Consecutive overload / recovery streaks determine when the requested factor changes.

A requested skip is not considered semantically complete merely because a control datagram was sent. If the returned `Camera` stream does not yet reflect the requested factor, the `Encoder` re-dispatches the same request after the configured three-frame interval. This preserves a bounded local retry plus application-level confirmation model without introducing transport retransmission.

Logged events remain:

```text
"WARMUP"
"IDLE"
"SKIP+1"
"SKIP-1"
"RETRY"
"INVALID"
```

All three repeated `B0` executions contain five `WARMUP` rows followed by 295 `IDLE` rows, with `frame_backlog = 0` throughout. Their steady-state maximum `workload_ratio` values are `0.186`, `0.181`, & `0.196`; none satisfies the overload condition, & every application frame retains `current_skip = 1`. Section 21.16 documents the active runtime overload case. The fair `O-OFF` reruns keep the browser-enabled runtime condition & every controller parameter unchanged while removing the `SFF1` geometry work; they remain below the controller transition threshold but raise the steady workload-ratio median from `0.165` to `0.419 / 0.391`. Active `P40` reaches `1.000` under elevated point-datagram pressure & records three complete increase / recovery cycles. Fair capture-enabled `OQ-OFF` remains below the controller transition frontier with a maximum `workload_ratio = 0.499` & preserves `current_skip = 1` for all 300 frames.

### 9.7 "CUDA" Memory Strategy

The projection pipeline proactively preallocates persistent device buffers, a designated "CUDA" stream, & precise timing events. Host "I420" output slots are similarly allocated at inception & formally registered with "CUDA".

Current buffering frameworks incorporate:

```text
H2D_CHUNK_POINTS = 65536
YUV_BUFFER_COUNT = 3
```

The point array is transferred asynchronously via designated chunks. Prior to each chunk transfer, the "CUDA" path asserts the capacity to invoke the `Encoder`'s "DPDK" polling callback. Following kernel initiation & subsequent copy-back, the worker continually polls while `cudaStreamQuery()` denotes the stream as incomplete.

This configuration strictly curtails the duration wherein "GPU" submission obstructs packet reception & circumvents repetitive device allocations along the measured pathway.

### 9.8 "CUDA" Projection Stages

The source-side pose carried by the `Camera` / `Encoder` path remains the neutral reference. Dynamic `User`-originated stance is intentionally applied later by `Decoder`; consequently, the `Encoder` can preserve its projection geometry independently from the reverse "Pose" service path.

For complete validated frames, `run_projection_pipeline()` consumes a ready `global_scale` & projected bounding-box centre irrespective of whether those values were produced by `SFF1` or by `compute_geometry_locally()`. The complete ON / OFF comparison therefore reaches the same "CUDA" branch with the same metadata shape; no second analytical projected-boundary formula is used for normal application frames.

The compatibility branch retained inside the projection helper serves warm-up / incomplete-invalid metadata safety only. When reached, it likewise reconstructs the projected frontier point by point:

```text
p'_i         = ( p_i - C ) * final_scale + ( 0, 0, CAMERA_DISTANCE )
projected B  = ( min_i( p'_i ) + max_i( p'_i ) ) / 2
global_scale = 1.10 * max( extent'_x / WIDTH, extent'_y / HEIGHT, extent'_z / WIDTH )
```

Consequently, the projection helper no longer introduces a distinct algebraic geometry convention between offloaded & locally recomputed complete frames.

The ensuing stages are structured as:

```text
asynchronous "H2D" transfer
fused point projection / colour conversion / z-buffer update
atlas packing
asynchronous "D2H" copy
```

Dynamic `User` "Pose" support is intentionally isolated downstream at `Decoder`; only a future migration of pose-dependent variables into the `Encoder` projection stage would require reassessment of this optimisation.

### 9.9 Six-View G-Buffer

The integrated "CUDA" kernel executes the following per point:

```text
object-centred scaling
normalised-coordinate construction
"BT.601" "RGB" -> "YUV" conversion
six orthographic face projections
depth quantisation
atomic z-buffer visibility arbitration
geometry / occupancy / texture updates
```

The directive `atomicMax()` mitigates visibility conflicts within the integer depth buffer. Depth arbitration itself is atomic, whereas the subsequent `Geometry` / `Occupancy` / `Texture` attribute stores do not form one indivisible transaction with that arbitration. Consequently, multiple threads mapping onto the same pixel are not assumed to yield bit-identical attribute-write ordering under arbitrary "GPU" scheduling, even though the retained validation evidence shows stable aggregate fidelity. This paradigm localises intensive point-parallel computations upon the "GPU", leaving only streamlined geometric control formulations resident on the host.

### 9.10 Atlas Geometry

The foundational projected view operates on:

```text
WIDTH  = 640
HEIGHT = 480
```

Faces receive padding to:

```text
FACE_W_PADDED = 640
FACE_H_PADDED = 512
```

& are structurally aligned in a four-by-three cross formation:

```text
CROSS_W = 2560
CROSS_H = 1536
```

Three identical, vertically stacked crosses represent:

```text
"Geometry"
"Texture"
"Occupancy"
```

yielding an encoded "I420" frame dimension of:

```text
ENCODER_W = 2560
ENCODER_H = 4608
```

The corresponding uncompressed "I420" buffer demands:

```text
TOTAL_YUV_SIZE = 17,694,720 B
```

per actively submitted frame.

### 9.11 "GPU" Timing Probes

The `Encoder` logs the following metrics:

```text
gpu_transfer_ms
gpu_kernel_ms
gpu_packing_ms
gpu_copyback_ms
host_overhead_ms
projection_ms
```

"CUDA" event chronologies isolate strictly asynchronous "GPU" operations, whereas `projection_ms` captures the full host-visible interval enveloping the projection call. Given that cooperative "DPDK" polling proceeds while the stream maintains an incomplete status, `host_overhead_ms` signifies residual host-visible time rather than acting as a definitive arithmetic-"CPU" kernel metric.

### 9.12 Persistent "FFmpeg" / "NVENC" Process & Pre-Roll

A singular, persistent "FFmpeg" process initiates concurrently with the experiment. The final validated live command retains the established rate-control configuration & adds only the two latency-oriented options that proved useful in isolation:

```text
"codec"        = hevc_nvenc
preset         = p2
tune           = ull
rate control   = cbr
target bitrate = 10M
buffer size    = 20M
"GOP"          = 15
forced "IDR"   = 1
delay          = 0
flush packets  = 1
```

No additional `zerolatency` option is enabled. `-delay 0` reduces retained Encoder-side pipeline depth, while `-flush_packets 1` requests prompt muxer packet emission. The latter is applied only on the Encoder: adding an analogous flush option to the Decoder was explicitly tested & did not produce a reproducible latency / cadence improvement, so the Decoder remains on its established `low_delay` configuration without it.

Preceding the measurement of application frames, the `Encoder` submits private blank `I420` material in `FRAMES = 15` groups until Decoder readiness is observed at a group boundary. The value `15` therefore defines the pre-roll granularity rather than a fixed total count. All three `B0` codec-log pairs together with `BQ` & `OQ-OFF` require `90` private frames before the `300` genuine application frames begin.

This pre-roll traffic traverses the ordinary Encoder / `SFF2` / Decoder media path under the reserved `FRAME_ID`, warming persistent `NVENC`, `MPEG-TS`, attribution, & `NVDEC` state without populating application telemetry.

A dedicated writer thread & three designated "I420" slots uncouple the projection phase from blocking interactions with "FFmpeg" stdin. In instances where all slots are committed, the `Encoder` perpetually services network / "codec" activities while awaiting slot emancipation.

### 9.13 Why "MPEG-TS" / "PES" Parsing Is Necessary

Pipe reads sourced from "FFmpeg" inherently fail to preserve distinct video-frame boundaries. Thus, the `Encoder` is obliged to reconstruct strictly structured 188-byte "MPEG-TS" packets from variable read sizes & identify precise video "PES" demarcations.

Under the validated persistent "FFmpeg" / "MPEG-TS" configuration, the detected video-"PES" frontier is used to attribute output to the oldest shot assigned to "FFmpeg". This enables the derivation of:

```text
encode_h265_ms = first_associated_PES - ffmpeg_input_start
```

The parser does **not** retain an entire encoded frame until the following "PES" boundary. Once a frame has been attributed, every complete `MEDIA_PAYLOAD_SIZE` group already accumulated in `mpeg_chunk` is emitted immediately; only the final residual fragment below the configured network-media capacity is held until the next video-"PES" frontier confirms that the preceding application frame has ended. The boundary therefore governs frame attribution & residual closure rather than acting as the release trigger for all compressed bytes.

This separation is central to the final latency behaviour. `-flush_packets 1` encourages "FFmpeg" / the "MPEG-TS" muxer to expose available material promptly, while the native parser progressively forwards already complete network chunks instead of introducing a second full-frame buffering policy. `encode_h265_ms` consequently remains a combined asynchronous "codec", scheduling, muxing, & pipe-delivery frontier; it is emphatically **not** promoted as an isolated "NVENC" hardware-kernel execution interval.

### 9.14 Encoder Output Packetisation

The `Encoder` systematically aggregates seven complete "TS" packets:

```text
7 * 188 B = 1316 B
```

& affixes the subsequent headers:

```text
cam_hdr
enc_hdr
```

before constructing an unencapsulated "UDP" packet destined for the `SFF2` proxy.

Notably, the `Encoder` issues no service-chain headers. Primary-path "NSH" state management persists unambiguously as an `SFF2` prerogative.

### 9.15 Encoder Telemetry — Complete Semantics

The final `Encoder` exports **69 fields**, making explicit the distinction among raw point reception, offloaded / fallback geometry, "GPU" projection, "codec" submission, asynchronous output attribution, "Temporal" control state, objective "luma" quality, & Tx acceptance:

```text
frame_id;rx_complete;tx_complete;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;codec_exit_time;node_exit_timestamp;original_points;rx_points;processed_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;codec_write_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;reference_jitter_ms;raw_queue_ms;render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_service_ms;encode_h265_ms;mse_y;psnr_y;ssim_y;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id`, `rx_complete`, `tx_complete`, `current_skip`, `event` | Native frame identity, input / compressed-output completion, active temporal factor, & controller state. |
| `yaw`, `pitch`, `zoom` | Source-projection pose retained for compatibility. Dynamic `User` pose is applied later by `Decoder`. |
| source / node timestamps | `Camera` source anchor, first `Encoder` arrival, first attributed "codec"-output frontier, & final "DPDK" egress. |
| `original_points`, `rx_points`, `processed_points` | Declared source population, unique received points, & population projected by the `Encoder`. |
| `rx_packets`, `tx_packets`, `payload_bytes`, `reference_size_bytes` | Raw input segmentation, compressed output segmentation, point payload, & reference comparison size. |
| throughput / bitrate fields | `internal_throughput_mbs` & `reference_throughput_mbs` are decimal MB / s over the real first-to-last raw-frame receive span; logical / network / reference bitrate fields remain expressed in Mbit / s. |
| `conversion_ms` | Network-to-host point conversion & deterministic placement accumulated over input packets. |
| `geometry_aggregation_ms`, `max_r_ms` | Local fallback geometry work. `geometry_aggregation_ms` includes raw aggregation plus the transformed projection-frontier pass in the complete local path, while `max_r_ms` remains separately attributed. Both remain zero when validated `SFF1` offload metadata is consumed. |
| `projection_ms` | Host-visible full "CUDA" projection interval. |
| `codec_write_ms` | Writer-side interval associated with submitting the projected frame to persistent "FFmpeg". |
| `active_tx_ms` | Active compressed "DPDK" egress calls. |
| `active_process_ms`, `reference_process_ms`, `total_processing_ms` | Measured active native work, reference-process comparator, & project-defined processing aggregate. They must not be blindly added to asynchronous "codec" latency. |
| `total_residency_ms`, `reference_residency_ms` | Measured / reference frame residence boundaries. |
| `node_efficiency_pct`, `reference_efficiency_pct` | Active-work ratios against measured / reference residence. |
| `gpu_transfer_ms`, `gpu_kernel_ms`, `gpu_packing_ms`, `gpu_copyback_ms`, `host_overhead_ms` | "CUDA" event-derived sub-stages & remaining host-visible projection overhead. |
| `camera_node_ms`, `e2e_latency_ms` | `Camera`-to-`Encoder` timing frontiers ending respectively at first Encoder arrival & final compressed-output node exit. |
| `schedule_delay_ms`, `inter_arrival_ms`, `instant_jitter_ms`, `desynced_jitter_ms`, `reference_jitter_ms` | Cumulative schedule drift, raw first-arrival interval, frame-ID-gap-aware deviation, smoothed jitter, & the active-temporal reference comparison. |
| `raw_queue_ms` | Frame readiness to `Encoder` service start. |
| `render_queue_ms` | Projection completion to "codec"-writer submission. |
| `workload_ewma_ms`, `workload_ratio` | Smoothed service signal & fraction of the active temporal budget. |
| `frame_backlog`, `codec_backlog` | Native pending frame count & observed writer / "codec" depth. "Codec" growth participates in overload / recovery evaluation. |
| `encode_service_ms` | Service interval used for `Encoder` workload characterisation. |
| `encode_h265_ms` | Genuine-frame "codec" submission to first attributed video-"PES" output frontier. |
| `mse_y`, `psnr_y`, `ssim_y` | Post-stream "luma" fidelity components; populated only with `QUALITY_CAPTURE = 1`. |
| `mpeg_bytes_generated` | Application-attributed "MPEG-TS" bytes before network packetisation. |
| `ffmpeg_write_calls`, `ffmpeg_write_eagain` | Pipe-write call count & nonblocking defensive retry observation. |
| Tx acceptance / starvation fields | Local compressed-output zero / partial accepts, resubmission attempts, & mbuf allocation failures. |

When `OFFLOAD_MODE_ENABLED` receives a final geometry snapshot whose `active_point_count` agrees with the assembled frame:

```text
geometry_aggregation_ms = 0
max_r_ms                = 0
```

The main workload variables are interpreted as:

```text
T_base           = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n              = EWMA_ALPHA * service_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio   = E_n / T_budget( skip )
```

`encode_h265_ms` remains an asynchronous "codec"-output latency & must not be arithmetically merged with `active_process_ms` or `total_processing_ms` as though the intervals were disjoint serial stages.

### 9.16 Encoder "End-of-Stream" Handling

Upon detecting "EOS", the `Encoder` initiates the finalisation of any eligible residual frame, drains lingering writer commitments & encoded outputs, seals the "FFmpeg" input channel, completes the parser exhaustion process, flushes residual "DPDK" output, & solely thereafter records the terminal telemetry data.

This sequential ordering remains essential because encoded bytes frequently linger within the "codec" / muxer pipeline long after the concluding application frame submission. Consequently, the terminal frame can routinely exhibit elevated residency or compressed-media delays in contrast to steady-state frames; thus, it demands analysis as an authentic tail-drain circumstance rather than suffering unceremonious deletion.

---

### 9.17 Optional Post-Stream "Luma" Quality Assessment

`Encoder` additionally exposes an explicit `QUALITY_CAPTURE` mode whose objective is to quantify coding distortion without introducing persistent disk activity into the measured streaming interval. When enabled, the node reserves bounded in-memory regions for both the authentic projected `Y` planes & the byte-preserved application-attributed "MPEG-TS" stream. Each genuine application frame contributes its "luma" reference through `write_reference()`, whilst encoded chunks are appended through `write_stream()`.

The streaming hot path therefore performs bounded memory copies exclusively; serialization to `reference_y.raw` & `encoded.ts`, alongside the subsequent objective analysis, occurs only after "EOS". Two isolated "FFmpeg" filter runs recover frame-associated `MSE-Y`, `PSNR-Y`, & `SSIM-Y`, whose results are merged back into `telemetry_encoder.csv`. Temporary analysis files are deleted only when every captured application frame has received complete indicators.

This mechanism deliberately distinguishes **real-time execution** from **offline quality evaluation**. The quality run must not be interpreted as a browser-interaction benchmark, while the interactive run deliberately leaves these columns unset in order to avoid contaminating the latency experiment with the additional fidelity capture. The quality path still performs sizeable "RAM" copies during streaming — notably one complete `Y` reference per real frame — so its sustained 30-fps result validates robustness under instrumentation but does not replace `QUALITY_CAPTURE = 0` as the clean performance condition.

### 9.18 Deferred Encoder Visual Snapshot

A separate compile-time visual diagnostic retains `DEBUG_FRAME_ID = 195`. When enabled, one `TOTAL_YUV_SIZE` snapshot is preallocated before streaming; the selected Encoder `I420` frame is copied into that memory region once, while all persistent `.i420` / `.pgm` creation is postponed until the stream, post-roll, optional quality assessment, & telemetry sequence have completed. The resulting files are:

```text
frame_195_input.i420
frame_195_geometry.pgm
frame_195_texture_y.pgm
frame_195_occupancy.pgm
```

The former synchronous file-system work is therefore absent from the selected frame's live path. A single in-memory copy necessarily remains if the three reusable `YUV_BUFFER_COUNT` slots are to continue circulating without changing their ownership model; for that reason visual validation is deliberately executed as a dedicated diagnostic run, while performance archives retain `DEBUG_VISUALS_DISABLED`.

## 🎞️ 10. Decoder — Persistent Hardware Decode, Reconstruction, & Dynamic "Pose"

### 10.1 Role

`Decoder` constitutes the second "SFC"-unaware application function. It receives standard "UDP" datagrams from the `SFF2` proxy, consumes the attributed "MPEG-TS" stream, reconstructs the projected point cloud, applies the latest `User` "Pose", & emits plain `dec_hdr + point_tx[]` packets back to `SFF2`.

### 10.2 Persistent "FFmpeg" / Hardware Decode

A persistent "FFmpeg" child is established using "CUDA" hardware acceleration & `hevc_cuvid`. The live command retains `-flags low_delay` with the existing `PROBE_SIZE` / `DURATION` settings; no Decoder-side `-flush_packets` is applied, because dedicated A / B runs did not establish a repeatable improvement when that option was added. Encoded "MPEG-TS" arrives through a bounded packet queue, while a dedicated writer thread feeds the child pipe. Both the writer & the child are explicitly pinned to logical Core `2`, avoiding scheduler migration into the "DPDK" / reconstruction core.

The internal "codec" queue & decoded-frame staging parameters are:

```
QUEUE_SIZE        = 16384
WRITE_BATCH_SIZE  = 65536
I420_BUFFER_COUNT = 3
FFMPEG_CPU        = 2
```

`Decoder` preserves application frame attribution separately from the raw "FFmpeg" stream chronology, as private warm-up / drain material prevents `ffmpeg.txt` rows from being interpreted as a one-to-one native frame index.

### 10.3 Pre-Roll & Readiness Coordination

`Decoder` startup is part of the deterministic launcher sequence. The decoder prepares its persistent "codec" state & exposes `/tmp/sfc-decoder-ready`; the `Encoder` waits for this readiness condition before the source path is ultimately admitted. This prevents initial stream traffic from racing an uninitialised decoding pipeline.

### 10.4 Reconstruction Pipeline

Each decoded `I420` image encodes the vertically stacked six-face `Geometry` / `Texture` / `Occupancy` representation emitted by `Encoder`. Reconstruction executes through a fixed "CUDA" path:

```
Host decoded "I420"
  -> "H2D" transfer
  -> 2 x 2 occupancy erosion
  -> per-face 3D reconstruction
  -> dynamic "Pose" kernel
  -> compact valid-point result
  -> "D2H" transfer
```

The native node records `arrived_points`, `eroded_points`, & `valid_points` independently, making representation / occupancy reduction distinct from network integrity. Specifically, `arrived_points` measures the loose-threshold occupancy population preceding erosion, `eroded_points` records the corresponding loose-threshold population remaining after erosion, & `valid_points` identifies the stricter reconstruction-threshold population actually admitted to the final point cloud. Thus, `eroded_points` denotes the post-erosion population rather than the number of points removed by erosion.

### 10.5 Dynamic "Pose" Application

`Decoder` receives a separate plain 24-byte "Pose" command from `SFF2`. A command is accepted only when its timestamp is newer than the currently retained control state. Reconstruction takes a stable snapshot of yaw, pitch, & zoom, applies the transformation in the "GPU" pipeline, & records `pose_control_ms` from the originating command timestamp to the measured application frontier.

"Pose" does not force `Encoder` reprojection. This preserves the architectural separation between the compression representation & end-user spatial interaction.

### 10.6 Cooperative Network Servicing

The main "DPDK" / reconstruction role remains bound to Core `4`, while "FFmpeg" I/O resides on Core `2`. Cooperative network callbacks continue processing datagrams around "codec" / "GPU" waits, limiting the opportunity for point or control traffic to accumulate merely because a frame is undergoing reconstruction.

### 10.7 Downstream Packetisation

Completed reconstruction is segmented at `POINTS_PER_PACKET = 80`. Every packet carries the 52-byte `dec_hdr`; the last packet is identifiable through exact point / sequence counts rather than a transport-level stream boundary. Empty valid frames retain a header-only representation with zero point population.

### 10.8 End-of-Stream Handling

"EOS" causes `Decoder` to close "codec" input, drain queued compressed material, consume remaining decoded outputs, issue the post-roll completion marker expected by `Encoder`, transmit a downstream terminal packet, & persist telemetry only after the attributed application state has been resolved.

### 10.9 Decoder Telemetry

The final 62-column schema is:

```text
frame_id;rx_complete;tx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_media_bytes;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;arrived_points;eroded_points;valid_points;erosion_ms;reconstruction_ms;pose_ms;reconstruction_pipeline_ms;tx_duration_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;pose_control_ms;codec_queue_ms;frame_queue_ms;codec_backlog;decode_service_ms;decode_h265_ms;ffmpeg_write_calls;ffmpeg_write_failures;codec_queue_drops;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```
The `Decoder`-specific indicators isolate compressed-media ingestion, hardware-decoding behaviour, reconstructed-geometry processing, dynamic pose application, internal queueing, & output generation:

| Field / Group | Exact Semantics |
|---|---|
| `rx_media_bytes` | Application-attributed compressed "MPEG-TS" bytes received for the corresponding frame prior to hardware decoding. `internal_throughput_mbs` includes the logical `cam_hdr + enc_hdr` once per populated frame & divides that logical byte count by the real receive span; `reference_throughput_mbs` uses the same temporal denominator for its reference-size comparator. |
| `arrived_points` | Loose-threshold occupancy population recovered from the decoded representation prior to morphological erosion. |
| `eroded_points` | Loose-threshold occupancy population remaining after erosion. Despite the field name, this value records the post-erosion population, not the number of points removed. |
| `valid_points` | Final point population admitted by the stricter reconstruction threshold after erosion & emitted toward the downstream path. |
| `erosion_ms` | "GPU" interval consumed by the occupancy-erosion operation preceding final point reconstruction. |
| `reconstruction_ms` | "GPU" interval dedicated specifically to converting the valid projected representation back into three-dimensional point coordinates. |
| `pose_ms` | "GPU" interval required to apply the active `yaw`, `pitch`, & `zoom` transformation to the reconstructed geometry. |
| `reconstruction_pipeline_ms` | Complete host-observed reconstruction interval spanning device preparation, erosion, point reconstruction, "Pose" application, copy-back, & associated orchestration. |
| `gpu_transfer_ms` | Time attributed to the host-to-device transfer / preparation operations required before reconstruction kernels can execute. |
| `gpu_copyback_ms` | Time required to recover the completed reconstructed point population from device-visible memory to the host-side output representation. |
| `host_overhead_ms` | Portion of `reconstruction_pipeline_ms` not directly accounted for by the individually measured "GPU" transfer, erosion, reconstruction, pose, & copy-back intervals. |
| `pose_control_ms` | Delay from the timestamp of a newly accepted `User` "Pose" directive to completion of its corresponding transformation inside the reconstruction pipeline. It is populated only when an actual pending pose generation is measured. |
| `inter_arrival_ms` | Raw first-media-packet interval between successive observed application frames. The initial frame reports `0`, while jitter compares the measured interval with the frame-ID-derived expected spacing. |
| `codec_queue_ms` | Delay from the first arrival of the attributed compressed frame at `Decoder` until the associated media begins submission toward the persistent "FFmpeg" decoder. |
| `frame_queue_ms` | Delay between availability of a decoded frame & the instant at which the native reconstruction service begins processing that decoded output. |
| `codec_backlog` | Maximum compressed-packet queue occupancy observed while receiving the application frame, exposing pressure between "DPDK" ingestion & the dedicated "FFmpeg" writer. |
| `decode_service_ms` | Host-side decoded-output service span beginning when `Decoder` starts retrieving a genuine post-pre-roll "I420" frame from persistent "FFmpeg" output & ending when the complete decoded frame buffer becomes available. |
| `decode_h265_ms` | Broader "H.265" latency measured from the first attributed compressed-data submission toward "FFmpeg" until the corresponding decoded frame becomes available. It therefore includes "codec"-side waiting in addition to the immediate decode service interval. |
| `ffmpeg_write_calls` | Number of writer-side pipe submission operations attributed to the frame while feeding the persistent "FFmpeg" process. |
| `ffmpeg_write_failures` | Number of unsuccessful persistent-decoder pipe write operations attributed to the frame. |
| `codec_queue_drops` | Number of compressed packet jobs attributed to the frame that could not be admitted to the bounded decoder "codec" queue. |
| `reference_process_ms` | `Decoder` reference-work comparator formed from `decode_service_ms` plus the reconstruction pipeline with `pose_ms` removed, thereby exposing the processing cost independently from active `User`-driven "Pose" transformation. |
| `reference_residency_ms` | Reference residency terminating at reconstruction completion rather than final downstream packet transmission. |
| `reference_efficiency_pct` | Ratio between `reference_process_ms` & `reference_residency_ms`, retained as a comparison frontier distinct from the complete node-efficiency definition. |
| `total_processing_ms` | Full frame-associated `Decoder` interval extending from the first attributed "codec" submission to the final node-exit frontier. Unlike `active_process_ms`, it intentionally includes intervening asynchronous waiting periods. |

### 10.10 Deferred Decoder Visual Snapshot

The Decoder mirrors the Encoder diagnostic without introducing persistent I / O into the reconstructed-frame service interval. With `DEBUG_VISUALS_ENABLED`, the fully decoded frame `195` is copied once after its native evaluation & before its reusable `I420` slot is released; serialization occurs only after "codec" input closure, output drain, downstream "EOS" dispatch, & telemetry export. The artefacts are:

```text
frame_195_output.i420
frame_195_geometry.pgm
frame_195_texture_y.pgm
frame_195_occupancy.pgm
```

No additional debug thread is introduced. On the current fully allocated host such a thread would merely relocate contention onto another scheduled context, while eliminating the remaining one-off "RAM" copy would require changing the three-slot lifetime or retaining an application buffer beyond its present ownership frontier. The validated design therefore keeps diagnostics methodologically separate rather than complicating the real-time Decoder path.

---

## 🧭 11. SFF3 — Final Aware Boundary & Reverse "Pose" Classifier

### 11.1 Primary-Path Role

`SFF3` receives the reconstructed primary stream exclusively from `SFF2`. It expects the base service state:

```
MAIN_SPI        = 100
MAIN_SI_DECODER = 253
```

The node validates "Ethernet" / "IPv4" / "UDP" / service fields, enforces frame sequence & point-count consistency, removes the 8-byte service envelope, rewrites the local `SFF3`-to-`User` network header, & forwards the unchanged `dec_hdr + point_tx[]` application body.

### 11.2 Reverse "Pose" Classification

The `User`-facing port also accepts a plain 24-byte `pose_payload`. `SFF3` validates the exact packet size & command fields, then constructs the reverse aware envelope:

```
POSE_SPI = 300
POSE_SI  = 255
```

The original application command is preserved byte-for-byte behind the service base so that `SFF2` can decapsulate it for `Decoder` without translating pose semantics.

### 11.3 Backpressure & Integrity

`SFF3` uses the same 4096-descriptor queues, 32-packet burst size, & bounded zero-accept policy as the remaining native chain. Its telemetry explicitly distinguishes primary Rx completion from Tx completion, which proved essential for diagnosing receiver-side backpressure during development. The repeated `B0` executions & both quality controls complete every admitted primary frame at this boundary with zero protocol errors, zero partial accepts, & zero Tx zero-accept events. `P40` is the deliberate exception in the sensitivity campaign: it exposes final-hop Tx pressure & is retained as a stress-boundary result rather than folded into the nominal completion statement.

### 11.4 Telemetry

`telemetry_sff3.csv` contains 41 fields:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

`cycle_occupancy_pct` preserves the same residence-to-cycle definition adopted by the other `SFF` stages, remaining distinct from the active-work `node_efficiency_pct`.

---

## 🖱️ 12. User — Final Reassembly, Interaction, Web Publication, & Quality Capture

### 12.1 Native End-Device Role

The native `User` process is the terminal "DPDK" consumer & the origin of the reverse "Pose" command. It validates `SFF3`-provided plain "UDP" packets, reassembles exact `dec_hdr` sequences, converts point coordinates to host representation, gathers end-to-end telemetry, & exposes complete snapshots through shared memory rather than binding the "DPDK" loop directly to browser networking.

### 12.2 Shared-Memory Publication

Two shared objects are used:

```
/dev/shm/frame.bin
/dev/shm/ctrl.bin
```

`frame.bin` contains the 72-byte `web_hdr` followed by the current `host_point[]` payload. An odd / even sequence marker prevents the bridge from copying a frame while the native process is still mutating it. `ctrl.bin` provides the reverse command / acknowledgment channel.

In `QUALITY_CAPTURE = 1`, frame publication & sequence updates are intentionally disabled because the "Web" path is absent. The same native reassembly logic remains active, preserving a common application contract across validation modes.

### 12.3 Non-Blocking "Pose" Dispatch & Returned-State Confirmation

A browser "Pose" query receives a monotonically increasing command identifier. The native process records the dispatch timestamp, requested yaw / pitch / zoom, first returning reference observation, first matching applied frame, & optional browser photon acknowledgment. Dispatch itself is non-blocking from the user-interaction perspective.

If an active command has not yet become observable in the returned stream, `User` re-presents it every:

```
RETRY_FRAMES = 3
```

This application-level re-presentation preserves the "UDP" transport model while protecting command observability from a transient lost control datagram.

### 12.4 Asynchronous "Python" / "WebSocket" Bridge

The bridge executes independently from the "DPDK" loop. Each connected peer owns:

```
asyncio.Queue( maxsize = 1 )
one frame_ready acknowledgment gate
```

`frame_loop()` observes only the shared sequence identifier & enqueues a lightweight availability token. It does **not** copy point payloads pre-emptively. `send_loop()` waits until the browser has acknowledged the preceding rendered frame, then copies whichever completed shared snapshot is latest at that moment. Consequently, stale unsent geometries are discarded naturally rather than accumulating as a "FIFO" backlog.

The "WebSocket" server runs with:

```
compression = None
```

while the "HTTP" server occupies an independent thread. In the final interactive launcher, this "Python" bridge runs on Core `2` with `nice -n 5`, deliberately giving the shared `Decoder` "codec" / writer work superior scheduler priority on that logical "CPU".

### 12.5 Three.js Viewer

The viewer employs `Three.js r128` with `OrbitControls`. Geometry storage expands in blocks of `65536` points, while the "GPU"-facing representation uses compact `Float32` "XYZ" & normalised `Uint8` "RGB" attributes. Rendering is event-driven through `requestAnimationFrame()`; there is no unconditional continuous animation loop.

Controls are:

```
"W" / "S" -> pitch
"A" / "D" -> yaw
"+" / "-" -> zoom
"R"       -> reset
```

Each angular step is `pi / 36` ( 5 degrees ) & each zoom modification uses a `1.05` multiplicative factor. A `streamReady` gate prevents a pose from being sent before the first genuine reconstructed frame establishes the active baseline; a locally accumulated pending "Pose" is dispatched immediately afterward.

### 12.6 Browser Acknowledgment & "Command-to-Photon"

The browser records `performance.now()` when a command is issued. When a subsequently received frame carries the matching command identifier, that timestamp remains associated until the frame has been submitted through the matching render pass. Immediately after `renderer.render()` returns, the viewer calculates the project-defined "Command-to-Photon" value, deletes the command entry, & sends a "JSON" acknowledgment containing the rendered frame, command, & "CTP" value.

The name is retained for continuity with the interactive metric, but its exact frontier is a **browser-side render-completion proxy** rather than a physical photon-to-eye measurement: no display compositor, "VSync", scan-out, panel-response, or photodiode instrumentation is included. This acknowledgment serves two purposes — telemetry & release of the one-frame-in-flight Web gate — without converting "Pose" dispatch into a blocking request / response interface.

### 12.7 Quality Capture & `Gauge` Synchronization

When `QUALITY_CAPTURE = 1`, no "Python" bridge, "HTTP" server, "WebSocket" server, or browser process is launched. A 1-GiB in-memory `quality_buffer` receives complete reconstructed frame records by copying the stable `web_points` content after native reassembly. Persistent serialization to `results.bin` occurs only after "EOS".

The terminal protocol is:

```
User closes / serializes quality capture
  -> User writes telemetry_user.csv
  -> User creates /tmp/sfc-user-quality
  -> Gauge starts offline analysis
  -> Gauge merges geometry metrics into telemetry_user.csv
  -> Gauge creates /tmp/sfc-user-done
  -> User prints final "End of stream detected..."
```

`Gauge` runs serially. It first reverses the applied pose, then applies statistical filtering & robust "ICP" alignment before calculating symmetric nearest-neighbour metrics. Its fixed parameters include:

```
VOXEL_MM        = 1.820
ICP_RADIUS      = 200.0 / VOXEL_MM
ICP_REPETITIONS = 30
ICP_POINTS      = 60000
OUT_K           = 20
OUT_STD         = 2.0
```

`ICP_POINTS = 60000` limits only the alignment fit: the reconstructed cloud is deterministically sampled by linear index up to 60,000 points, the reference fit up to 120,000 points, & the resolved rigid transform is subsequently applied to the complete reconstructed cloud. The final symmetric nearest-neighbour metrics are then evaluated against the full aligned reconstructed & full reference populations, so the reported quality values are not 60,000-point metric estimates.

All `cKDTree` operations explicitly employ one worker. The serial design is a post-stream methodological choice favouring determinism & simplicity; it does not execute concurrently with the measured 300-frame "DPDK" stream.

`Gauge` defines the expected assessment population from `User` rows satisfying `rx_complete = 1`. A frame contributes a completed objective result only when its capture record & reference geometry are both available & the metric pipeline returns successfully. The temporary `results.bin` capture is removed exclusively when the completed population equals the expected one; each retained quality control therefore evaluates all 300 eligible frames rather than silently tolerating missing geometric assessments.

### 12.8 User Telemetry

The final 48-column schema is:

```text
frame_id;rx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;arrived_points;eroded_points;valid_points;rx_points;rx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;arrival_pct;erosion_pct;valid_pct;web_publish_ms;web_ack_ms;active_process_ms;total_residency_ms;node_efficiency_pct;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;cmd_id;reference_cmd_ms;cmd_apply_ms;cmd_photon_ms;quality_save_ms;mean_error;geom_rmse;chamfer;hausdorff;mean_mm;rmse_mm;chamfer_mm;hausdorff_mm
```

The final eight geometry columns remain unset in the interactive run & are populated by `Gauge` only in quality mode.

Moreover, `User`-specific indicators describe the terminal reconstruction population, browser-publication frontier, interactive "Pose" chronology, & optional post-stream objective-quality evaluation:

| Field / Group | Exact Semantics |
|---|---|
| `arrival_pct` | Percentage of the original source-frame population represented by the reconstruction candidates reported by `Decoder`, computed as `arrived_points / original_points * 100`. |
| `erosion_pct` | Percentage of loose-threshold reconstruction candidates surviving the `Decoder` erosion stage, computed as `eroded_points / arrived_points * 100`. |
| `valid_pct` | Final valid reconstructed population expressed against the original source cloud, computed as `valid_points / original_points * 100`. |
| `web_publish_ms` | Native `User` time required to finalise the stable shared-memory Web header & publish the completed frame through the odd / even sequence protocol. It remains zero during `QUALITY_CAPTURE = 1`, where Web publication is disabled. |
| `web_ack_ms` | Delay from native publication of a frame to reception by `User` of the corresponding browser render acknowledgment through the asynchronous control mapping. |
| `reference_e2e_ms` | `Camera`-to-`User` latency terminating when the complete reconstructed frame becomes available natively, before optional Web publication extends the final node-exit frontier. |
| `schedule_delay_ms` | Cumulative drift against the ideal frame-ID schedule, using `node_exit_timestamp` as the real completion frontier & the first observed shot arrival as the fixed session origin. |
| `inter_arrival_ms` | Raw interval between the first packets of successive completed-frame observations; the first accepted shot reports `0`, while jitter remains corrected by the actual frame-ID gap. |
| `cmd_id` | Identifier of the "Pose" directive whose requested state is first observed on the corresponding returned reconstructed frame. Zero identifies frames not associated with a newly matched directive. |
| `reference_cmd_ms` | Delay from native dispatch of a "Pose" directive to arrival of the first subsequently observed complete frame used as the command-reference frontier, independently from whether its pose already matches the request. |
| `cmd_apply_ms` | Delay from directive dispatch until the first complete returned frame whose `yaw`, `pitch`, & `zoom` values actually match the requested pose. |
| `cmd_photon_ms` | Browser-side "Command-to-Photon" proxy: interval from command generation in the viewer until immediately after the `renderer.render()` call for the first frame carrying the matching directive. It excludes compositor / scan-out / panel delay & is returned asynchronously to `User` through the Web acknowledgment mapping. |
| `quality_save_ms` | In `QUALITY_CAPTURE = 1`, time required to append the completed reconstructed point-cloud snapshot to the bounded in-memory quality capture buffer. No corresponding operation is performed in the interactive condition. |
| `mean_error` | Directed reconstructed-to-reference mean nearest-neighbour geometric error obtained after inverse-pose normalisation, statistical filtering, & robust "ICP" registration against the original reference frame. Populated by `Gauge` only after a quality run. |
| `geom_rmse` | Symmetric root-mean-square geometric deviation derived from reconstructed-to-reference & reference-to-reconstructed nearest-neighbour distances after registration. |
| `chamfer` | Symmetric Chamfer distance obtained as the sum of the two directional mean nearest-neighbour distances. |
| `hausdorff` | Symmetric Hausdorff distance, representing the maximum nearest-neighbour deviation observed in either geometric direction. |
| `mean_mm` | `mean_error` converted from dataset voxel coordinates to millimetres through `VOXEL_MM = 1.820`. |
| `rmse_mm` | `geom_rmse` expressed in millimetres. |
| `chamfer_mm` | Symmetric Chamfer distance expressed in millimetres. |
| `hausdorff_mm` | Symmetric Hausdorff distance expressed in millimetres. |

---

## 🖥️ 13. "CPU", "GPU", & Core-Constrained Execution

The host exposes four physical cores / eight logical "CPU" threads with sibling pairs:

```text
{ 0, 4 }
{ 1, 5 }
{ 2, 6 }
{ 3, 7 }
```

The final placement intentionally reuses sibling capacity across dissimilar roles while avoiding an additional core-consuming virtual switch:

| Logical Core | Final Role | Experimental Rationale |
|---:|---|---|
| `0` | `User` "DPDK"; in quality, serial `Gauge` **after "EOS"** | Terminal receive path remains isolated from the browser; offline `Gauge` reuses the same core only after streaming has ended |
| `1` | `Camera` | Source scheduling & packetisation; this logical core is now available because the former "OVS"-"DPDK" "PMD" was removed |
| `2` | `Decoder` "FFmpeg" child + writer; interactive `User` "Python" bridge at `nice +5` | "Codec" I/O is pinned; lower-priority bridge sharing is admitted only in NON-QUALITY |
| `3` | `SFF1` | Dedicated in-path geometry service function |
| `4` | `Decoder` "DPDK" / reconstruction role | Separates native packet / reconstruction work from "codec" pipe I/O |
| `5` | `Encoder` "DPDK" | Dedicated `Encoder` native processing core |
| `6` | `SFF2` | Dedicated four-port service proxy |
| `7` | `SFF3` + `Encoder` "FFmpeg" / writer | Final service boundary shares a sibling with asynchronous "codec" I/O; the measured `SFF3` load remains comparatively small |

Container cpusets are:

```text
Camera  : "1"
SFF1    : "3"
SFF2    : "6"
Encoder : "5,7" ( "DPDK" = 5, Encoder "codec" I/O = 7 )
Decoder : "2,4" ( "DPDK" = 4, Decoder "codec" I/O = 2 )
SFF3    : "7"
User    : "0,2" when QUALITY_CAPTURE = 0
User    : "0"   when QUALITY_CAPTURE = 1
```

The elimination of "OVS"-"DPDK" therefore removes the old dedicated "PMD" / auxiliary forwarding allocation from the methodology. The released scheduling capacity is consumed by the application chain itself, notably permitting `Camera` to occupy logical Core `1` directly. Since the host exposes eight logical "CPU" threads, eliminating one switch-only logical-core reservation makes `12.5 %` of the host's logical scheduling capacity available for application placement. This value describes **allocation opportunity**, not a measured 12.5 % reduction in aggregate cycles or power.

The resulting graph also exposes seven native placement units ( `Camera`, `SFF1`, `SFF2`, `Encoder`, `Decoder`, `SFF3`, `User` ) rather than the four coarse services of the reference architecture. Numerically this is a `75 %` increase in the number of graph-level placement units ( `( 7 - 4 ) / 4 = 0.75` ), but it should be interpreted as **orchestration granularity**, not as improved resource efficiency by itself. Each function has a dedicated telemetry / affinity profile, allowing a future orchestrator to move, replicate, or allocate resources to the responsible stage rather than relocating an entire monolithic `Encoder` or Client block.

No percentage claim concerning total "CPU" utilisation **against the reference implementation** is made from the supplied data. The two projects do not expose an identical per-core utilisation experiment, & the current deployment deliberately touches all eight logical threads through native, "codec", bridge, or housekeeping roles. A fair resource-efficiency comparison therefore requires a controlled co-located campaign reporting actual "CPU" time / utilisation, "GPU" occupancy, memory footprint, & energy under identical traffic. The current contribution is that such attribution is now possible per service function.

### Core `0` & Housekeeping

Core `0` remains the only non-isolated logical "CPU" when `isolcpus=1-7` is active. Assigning `User` "DPDK" to this core is an explicit experimental choice & must be retained in reproducibility records. Quality analysis is not concurrent with `User` streaming: `Gauge` begins only after `/tmp/sfc-user-quality` is created following "EOS".

### Why the Current Constraint Is Manageable

The final implementation mitigates pressure through native affinity, "GPU" projection / reconstruction, persistent "codec" processes, triple image buffering, cooperative network polling, bounded descriptor queues, latest-frame-only browser publication, & source-level temporal regulation. The host remains deliberately constrained; thus, core affinity is an integral experimental variable rather than an incidental deployment detail.

---

## 14. "HugePages" & Optional "CPU" Isolation

The current `init_all.sh` resets filesystem caches & configures:

```text
nr_hugepages  = 1024
HugePage size = 2 MiB
total         = 2048 MiB ~= 2 GiB
```

The cache reset precedes topology construction. `Camera` subsequently establishes its own `WARM_MODE_ENABLED` condition, so the host reset & source warm-mode configuration remain separate reproducibility variables.

The optional isolation helper installs:

```text
isolcpus=1-7
```

leaving Core `0` as the sole non-isolated housekeeping "CPU". Enabling isolation requires a reboot:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot:

```bash
cat /proc/cmdline
```

must confirm the expected parameter. Default scheduling is restored through:

```bash
sudo ./disable_isolcpus.sh
sudo reboot
```

`init_all.sh` & `stop_all.sh` intentionally do not alter this reboot-tier state. "HugePages", `isolcpus`, "Docker" cpusets, native "DPDK" lcores, & "codec" affinity must therefore be reported independently for every benchmark.

---

## 15. Container & Direct "DPDK" "SFC" Environment

The common image is based upon:

```text
nvidia/cuda:12.2.0-devel-ubuntu22.04
"DPDK" 22.11.4 LTS
Ubuntu 22.04
```

The build installs the native compiler toolchain, "Meson" / "Ninja", "NUMA" dependencies, "FFmpeg", "Python", & the libraries required by the browser / quality helpers. The "Python" runtime includes compatible `websockets`, `numpy`, & `scipy` packages.

Native service containers ordinarily execute with:

```text
--privileged
--net none
```

The sole exception is the interactive `User` container, which receives the "Docker" bridge plus ports `8080` / `9999` so that "HTTP" / "WebSocket" traffic can reach the local browser bridge. In quality mode those ports & the bridge are omitted.

Each service mounts:

```text
/dev/hugepages
/tmp
/shared
/app
```

### Build-Time Specialisation

The project treats compiler configuration, "CUDA" architecture, "GPU" model / driver, "FFmpeg" / "NVENC" stack, & "DPDK" version as constituent elements of the experimental condition. The final Encoder / Decoder Makefiles use the following principal flags, in addition to the applicable `pkg-config`-derived "DPDK" include / link arguments:

```text
"C++"  : -O3 -march=native -ffast-math -funroll-loops -std=c++14 -Wall
"CUDA" : -O3 -arch=sm_61 -std=c++14 -Xcompiler -fPIC
```

The `-arch=sm_61` specification fixes the generated "CUDA" device-code target to Compute Capability `6.1`, rather than defining an architecture-neutral numerical implementation. Consequently, cross-platform bit-level equivalence cannot be presumed when a comparison changes the "GPU" architecture, "CUDA" compiler / toolkit, or execution-driver environment. The same reproducibility constraint applies independently to the hardware `H.265` path, whose `NVENC` / `NVDEC` execution additionally depends upon the available "GPU", NVIDIA driver, "FFmpeg" build, & hardware codec generation.

These factors chiefly constrain **bit-level fidelity reproducibility** across hardware / toolchain changes: small numerical or codec-output differences cannot be interpreted independently of the concrete "GPU", driver, "CUDA", "FFmpeg", & hardware-"codec" environment. The runtime architectural comparison is addressed separately through the repeated B0 system result & the paired in-network placement ablation.

### Direct "SFC" Topology

`setup_topology.sh` now performs a deliberately minimal host operation: it removes stale `/tmp/sfc-*` / `/tmp/vh-*` endpoints & declares the six expected direct adjacencies. No "OVS" bridge, OpenFlow rule, "PMD" core, mirror port, or virtual-switch socket-memory policy exists in the final launch path.

`start_microservices.sh` then starts `SFF2` first, waits for its four service-side sockets, starts `Decoder` & `Encoder`, waits for their readiness markers, attaches `SFF3`, then `User`, `SFF1`, & finally `Camera`. This order converts Unix socket availability into an explicit startup dependency rather than relying upon arbitrary sleep intervals.

In NON-QUALITY the launcher pauses before `Camera` until the remote viewer has been associated. In quality mode this interactive gate is skipped because the browser path is intentionally absent.

---

## 16. Repository Structure

The completed repository organisation is conceptually:

```text
Thesis/
├── README.md
├── docs/                              # Thesis / reference material
├── env/                               # Root-level "Python" environment for offline preparation
└── src/
    ├── infrastructure/
    │   ├── setup_topology.sh
    │   └── start_microservices.sh
    │
    ├── microservices/
    │   ├── base/
    │   │   └── Dockerfile
    │   ├── camera/
    │   │   ├── c/camera.c
    │   │   └── entrypoint.sh
    │   ├── sff1/
    │   │   ├── c/sff1.c
    │   │   └── entrypoint.sh
    │   ├── sff2/
    │   │   ├── c/sff2.c
    │   │   └── entrypoint.sh
    │   ├── encoder/
    │   │   ├── cpp/encoder.cpp
    │   │   ├── cu/encoder.cu
    │   │   ├── h/encoder.h
    │   │   └── entrypoint.sh
    │   ├── decoder/
    │   │   ├── cpp/decoder.cpp
    │   │   ├── cu/decoder.cu
    │   │   ├── h/decoder.h
    │   │   └── entrypoint.sh
    │   ├── sff3/
    │   │   ├── c/sff3.c
    │   │   └── entrypoint.sh
    │   └── user/
    │       ├── c/user.c
    │       ├── py/user.py
    │       ├── py/gauge/gauge.py
    │       ├── html/index.html
    │       └── entrypoint.sh
    │
    ├── shared/
    │   ├── data/loot/
    │   │   ├── original/              # Original PLY sequence
    │   │   ├── bin/                   # Header-less 16-B/point reference frames
    │   │   └── made/                  # Temporary quality-capture outputs
    │   ├── log/
    │   │   ├── converter/
    │   │   ├── camera/
    │   │   ├── sff1/
    │   │   ├── sff2/
    │   │   ├── encoder/
    │   │   ├── decoder/
    │   │   ├── sff3/
    │   │   └── user/
    │   └── py/
    │       ├── converter/converter.py
    │       └── gauge/gauge.py
    │
    ├── enable_isolcpus.sh
    ├── disable_isolcpus.sh
    ├── init_all.sh
    └── stop_all.sh
```

Exact deployment paths should follow the repository snapshot being executed; the tree above documents the logical ownership reflected by the supplied source & launcher files.

---

## 🧬 17. Dataset, "Python" Environment, & Offline Preparation

### 17.1 Research Dataset — 8i Voxelized Full Bodies ( "Loot" )

The designated experimental point-cloud resource relies upon the **"Loot" sequence traversing the 8i Voxelized Full Bodies ( 8iVFB v2 ) dataset**, graciously provided by 8i Labs & exhaustively catalogued through the "JPEG" Pleno database.

The foundational dataset establishes four diverse dynamic full-body sequences:

```text
longdress
loot
redandblack
soldier
```

Each sequence presents a comprehensive human subject meticulously captured via 42 "RGB" cameras systematically configured within 14 clusters, capturing at 30 frames / s for an approximate duration of 10 s. The depth-10 structure necessitates a `1024 x 1024 x 1024` voxel grid, wherein "RGB" colour attributes are rigorously assigned to occupied voxels.

The focal experiment employs the absolute 300-frame depth-10 "Loot" sequence:

```text
loot_vox10_1000.ply
...
loot_vox10_1299.ply
```

The rigorously documented prepared "Loot" dataset snapshot confirms the following metrics:

| **Dataset quantity**                   | **Current "Loot" snapshot**      |
| -------------------------------------- | -------------------------------- |
| Frames                                 | `300`                            |
| Total points                           | `238,146,391`                    |
| Mean points / frame                    | `793,821`                        |
| Minimum points / frame                 | `770,822`                        |
| Maximum points / frame                 | `835,458`                        |
| Aggregate source "PLY" footprint         | `5,144,378,340 B` ( `5.144 GB` ) |
| Aggregate generated "BIN" footprint      | `3,810,342,256 B` ( `3.810 GB` ) |
| "BIN" footprint reduction vs. source "PLY" | `25.93 %`                        |
| Mean offline conversion time           | `7.124 s / frame`                |

The entire dataset remains publicly accessible via the "JPEG" Pleno database compliant with the attached 8i license protocols. The formally required academic citation demands:

> E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, *8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset*, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.

Repository stakeholders must imperatively consult the primary dataset portal & corresponding license prior to any utilisation or subsequent redistribution:

```text
[https://plenodb.jpeg.org/pc/8ilabs/](https://plenodb.jpeg.org/pc/8ilabs/)
```

### 17.2 Repository Data Policy

Conspicuously, neither the authentic `.ply` frames nor the procedurally generated `.bin` frames hold presence within this repository's commit history.

This outcome is strictly intentional: the exhaustive "Loot" "PLY" series scales to approximately `5.14 GB` locally, while the compact binary derivative persists at roughly `3.81 GB`. Excluding both manifestations from Git rigorously guarantees a streamlined repository architecture & actively prevents fundamental source-control mechanisms from succumbing to immense experimental data weights.

Consequently, the repository securely houses the **code, data schematics, procedural conversion paradigms, & overarching telemetry**, operating on the presumption that voluminous dataset artefacts will be either independently procured or locally generated.

This capacity-driven repository policy strictly operates independent of the official dataset licence. Any local replication or expansive redistribution concerning 8i assets remains unequivocally bound by the explicit licence appended to the original dataset.

### 17.3 Binary Representation Used by the Camera

The offline converter methodically transfigures every "PLY" frame into a seamless, header-less array congruent with `point_tx`:

```text
float32 x
float32 y
float32 z
uint8   r
uint8   g
uint8   b
uint8   padding
```

Consequently:

```text
bytes_per_point = 16
```

This robust transformation effectively isolates "PLY" interpretive parsing & distinct per-field numeric conversions from the `Camera`'s high-frequency streaming conduit. It definitively serves as a **storage / parsing preparatory sequence**, unequivocally void of aspirations mimicking compression algorithms grounded in rigorous information theory or explicit rate-distortion frameworks.

The prevalent scale factor anchored within the repository mandates:

```text
SCALE_FACTOR = 1.0
```

### 17.4 "Python" Environment

The definitive root-level directory:

```text
env/
```

contains the precise "Python" virtual configuration requisite for the current offline utilities, most notably the point-cloud translation apparatus.

Activation from the repository root unfolds as:

```bash
source env/bin/activate
```

The existing README snapshot asserts the requisite converter dependencies encompass:

```text
numpy
plyfile
```

Should an environment reconstruction become necessary:

```bash
python3 -m venv env
source env/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy plyfile
```

Libraries encompassing `pandas` & `matplotlib` definitively lack prerequisite status regarding the execution of the primary documented converter. Although exceptionally competent regarding elevated telemetry analysis, they decisively remain extraneous to the native "DPDK" operational pathway.

### 17.5 Offline Converter

The standard execution paradigm manifests as:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The converter operates exclusively as an **offline preparation stage**. The resultant elapsed chronology, encompassing both "PLY" ingestion & "BIN" extrusion, must definitively eschew amalgamation with `Camera`, `SFF`, `Encoder`, "CUDA", or explicit "codec" latency quantifications.

Nonetheless, the converter telemetry presents substantial utility for replicability parameters, flawlessly tracking the strict frame population & precisely contrasting the source against the generated data footprint prevalent throughout the experiment. The definitive source-configured schema embodies:

| **Metric**         | **Unit / Type** | **Exact Meaning**                                                                                                                                                                      |
| ------------------ | --------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `filename`         | string          | Source `.ply` file subjected to offline processing operations.                                                                                                                         |
| `status`           | string          | `success` dictates the successful culmination of the conversion path lacking exception; alternatively `error`.                                                                         |
| `num_points`       | points          | Aggregate vertices extracted from the "PLY" `vertex` entity & transcribed to the designated fixed-width binary format.                                                                   |
| `read_ascii_ms`    | ms              | Absolute wall-clock interval consumed exclusively within `PlyData.read( file_path )`.                                                                                                  |
| `write_bin_ms`     | ms              | Absolute wall-clock interval spent writing the already established contiguous `network_array` to the `.bin` output.                                                                    |
| `conversion_ms`    | ms              | Comprehensive file interval spanning parsing, coordinate / colour conversion, structural record formation, output writing, & associated per-file management.                           |
| `size_ascii_bytes` | bytes           | Exact dimensions of the foundational `.ply` artefact, sourced through `os.path.getsize()`.                                                                                             |
| `size_bin_bytes`   | bytes           | Exact dimensions of the synthetic fixed-width `.bin` artefact, similarly sourced through `os.path.getsize()`.                                                                          |

The telemetry name `read_ascii_ms` is retained for compatibility, but the implementation times `PlyData.read( file_path )` regardless of whether the source `.ply` encoding is textual or binary. `conversion_ms` is purposefully broader in scope than the strict calculation of `read_ascii_ms + write_bin_ms`: it additionally covers point-array extraction, `SCALE_FACTOR` application, numeric casting, padding construction, fixed-width record formation, & associated per-file control work. Consequently, it must **never** be interpreted merely as the rudimentary summation of the two distinct I / O probes.

The robust streaming outcomes explored within Section 21 inaugurate exclusively from the synthesised binary array. Ergo, Offline Converter chronologies are systematically expunged from all `Camera` / `SFF1` / `SFF2` / `Encoder` / `Decoder` / `SFF3` / `User` latency metrics & hold relevance strictly parallel to the correlative Converter telemetry tied to the dataset-preparation epoch.

---

## 🚀 18. Running the Experiment

### 18.1 Obtain & Prepare the Dataset

Acquire the 8iVFB v2 "Loot" sequence according to its original licence, place frames `loot_vox10_1000.ply` through `loot_vox10_1299.ply` beneath the expected `shared/data/loot/original` directory, & run the offline Converter before any streaming benchmark.

### 18.2 Optional: Enable "CPU" Isolation

From `src/`:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot, verify `/proc/cmdline` before collecting results.

### 18.3 Select the Validation Condition

`start_microservices.sh` exposes the top-level experiment switch:

```bash
QUALITY_CAPTURE="0"
```

Use `0` for the interactive runtime benchmark & `1` for the isolated objective-quality run. This value is intentionally explicit rather than silently inferred.

**NON-QUALITY ( `QUALITY_CAPTURE = 0` )**

```text
User cpuset          = "0,2"
User "DPDK"          = Core 0
"Python" bridge      = Core 2, nice +5
"HTTP" / "WebSocket" = enabled
viewer               = enabled
launch waits for viewer before Camera
Encoder PSNR / SSIM capture = disabled
User Gauge                  = disabled
```

**QUALITY ( `QUALITY_CAPTURE = 1` )**

```text
User cpuset          = "0"
User "DPDK"          = Core 0
"Python" bridge      = disabled
"HTTP" / "WebSocket" = disabled
viewer               = absent
Camera starts after native readiness, without "ENTER" gate
Encoder "luma" quality capture = enabled
User geometry capture          = enabled
Gauge                          = serial, Core 0, strictly post-"EOS"
```

### 18.4 Start the Environment

From `src/`:

```bash
sudo ./init_all.sh
```

This resets caches, allocates 1024 2-MiB "HugePages", clears stale direct sockets, builds the base image, starts native containers in dependency order, & launches `Camera` only after the required readiness conditions are satisfied.

### 18.5 Interactive Viewer

For NON-QUALITY, associate the browser with the exposed `User` "HTTP" endpoint before confirming the launcher prompt. Once connected, `Camera` begins the 300-frame source sequence & the browser can issue pose commands through buttons or keyboard controls.

### 18.6 Inspect the Active Nodes

```bash
sudo docker ps
sudo docker logs camera
sudo docker logs sff1
sudo docker logs sff2
sudo docker logs encoder
sudo docker logs decoder
sudo docker logs sff3
sudo docker logs user
```

### 18.7 Stop the Experiment

```bash
sudo ./stop_all.sh
```

The shutdown script removes all seven containers, clears direct "DPDK" socket / runtime files, removes the per-node `/dev/hugepages/*map_*` artefacts, releases "HugePages", synchronises persistent state, & drops caches. It deliberately does not reverse `isolcpus`, because "CPU" isolation is a reboot-tier experimental condition.

---

## 19. Entrypoint Execution Model

Every native node is compiled / launched from its mounted `/app` source tree rather than from a stale binary embedded in the image. `start_microservices.sh` supplies a node-specific `DPDK_CORE`, cpuset, optional "GPU" permission, & optional `QUALITY_CAPTURE` environment.

The supplied launch scripts make the build boundary explicit. The `User` entrypoint compiles `/app/c/user.c` directly through `gcc -O3` with the `pkg-config`-resolved "DPDK" flags & `-lm`, whereas the `Encoder` entrypoint executes `make clean` followed by `make all` before launching `/tmp/encoder_dpdk`. The supplied Encoder Makefile defaults `CPU_ARCH = native` & `CUDA_ARCH = sm_61`, applying the "C++" / "CUDA" optimisation profile documented in Section 15. Rebuilding immediately before process launch prevents a stale application binary from silently surviving a source-level campaign change.

The `User` entrypoint additionally owns the auxiliary presentation / quality lifecycle:

```
QUALITY_CAPTURE = 0
  -> start "Python" "HTTP" / "WebSocket" bridge on Core 2 with nice +5
  -> start User "DPDK" on Core 0

QUALITY_CAPTURE = 1
  -> start a background shell gate waiting for /tmp/sfc-user-quality
  -> start User "DPDK" on Core 0
  -> after User "EOS" / capture serialization, launch serial gauge.py on Core 0
  -> Gauge writes /tmp/sfc-user-done
  -> User exits its quality wait & prints the final "EOS" state
```

The "BLAS" / numeric thread environment for `Gauge` is fixed to one thread ( `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, & `NUMEXPR_NUM_THREADS` all equal `1` ). This prevents the offline assessment from silently spawning a larger "CPU" pool.

---

## 📊 20. Native Telemetry Files

The final repository produces a complete per-node observation chain:

| Component | File | Final column count |
|---|---|---:|
| Converter | `log/converter/telemetry_converter.csv` | `8` |
| `Camera` | `log/camera/telemetry_camera.csv` | `31` |
| `SFF1` | `log/sff1/telemetry_sff1.csv` | `43` |
| `SFF2` Route 0 | `log/sff2/telemetry_sff1_enc.csv` | `43` |
| `Encoder` | `log/encoder/telemetry_encoder.csv` | `69` |
| `SFF2` Route 1 | `log/sff2/telemetry_enc_dec.csv` | `43` |
| `Decoder` | `log/decoder/telemetry_decoder.csv` | `62` |
| `SFF2` Route 2 | `log/sff2/telemetry_dec_sff3.csv` | `43` |
| `SFF3` | `log/sff3/telemetry_sff3.csv` | `41` |
| `User` | `log/user/telemetry_user.csv` | `48` |
| `Encoder` "codec" | `log/encoder/ffmpeg.txt` | `vstats` text |
| `Decoder` "codec" | `log/decoder/ffmpeg.txt` | `vstats` text |

### 20.1 Exact ".csv" Schema Reference

**`Camera` — 31 columns**

```text
frame_id;selected;tx_complete;current_skip;last_control_frame;temporal_control_ms;camera_send_timestamp;tx_start_timestamp;inter_departure_ms;tx_points;tx_packets;payload_bytes;reference_size_bytes;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;disk_io_ms;serialization_ms;tx_duration_ms;active_tx_ms;active_process_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`SFF1` — 43 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;geometry_aggregation_ms;max_r_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`SFF2` Routes 0 / 1 / 2 — 43 columns each**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_media_bytes;tx_media_bytes;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`Encoder` — 69 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;codec_exit_time;node_exit_timestamp;original_points;rx_points;processed_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;codec_write_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;reference_jitter_ms;raw_queue_ms;render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_service_ms;encode_h265_ms;mse_y;psnr_y;ssim_y;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`Decoder` — 62 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_media_bytes;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;arrived_points;eroded_points;valid_points;erosion_ms;reconstruction_ms;pose_ms;reconstruction_pipeline_ms;tx_duration_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;pose_control_ms;codec_queue_ms;frame_queue_ms;codec_backlog;decode_service_ms;decode_h265_ms;ffmpeg_write_calls;ffmpeg_write_failures;codec_queue_drops;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`SFF3` — 41 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`User` — 48 columns**

```text
frame_id;rx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;arrived_points;eroded_points;valid_points;rx_points;rx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;arrival_pct;erosion_pct;valid_pct;web_publish_ms;web_ack_ms;active_process_ms;total_residency_ms;node_efficiency_pct;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;cmd_id;reference_cmd_ms;cmd_apply_ms;cmd_photon_ms;quality_save_ms;mean_error;geom_rmse;chamfer;hausdorff;mean_mm;rmse_mm;chamfer_mm;hausdorff_mm
```

#### 20.1.1 Cadence & Reference-Throughput Semantics

Cadence is now observable without reconstructing it retrospectively from unrelated residence fields. `Camera.inter_departure_ms` measures send-start to send-start across successive transmitted frames, while every receiving native stage exposes `inter_arrival_ms` from first valid packet to first valid packet. The first sample is explicitly `0`. Neither field divides the elapsed interval by a frame-ID gap: a deliberately skipped source frame therefore remains visible in the raw spacing. Jitter is handled separately, comparing the real interval with `( frame_id - previous_frame_id ) / TARGET_FPS`; intentional temporal selection is consequently not misclassified as network jitter.

The reference-throughput correction follows the same boundary discipline. `SFF1`, all three `SFF2` routes, `SFF3`, & `Decoder` compute `reference_throughput_mbs` from logical reference bytes divided by the **real first-to-last receive duration** of the frame, using decimal `1,000,000 B = 1 MB`. The corresponding `internal_throughput_mbs` uses the same receive span with measured logical bytes. `reference_bitrate_mbps` remains a distinct Mbit / s quantity based upon effective frame rate & must not be compared numerically as though it were another spelling of throughput.

### 20.2 "FFmpeg" `vstats` Field Semantics

The `Encoder` & `Decoder` `ffmpeg.txt` files are diagnostic "codec" chronologies, not one-to-one application-frame logs. All three `B0` repetitions, `BQ`, & `OQ-OFF` contain `395` statistics rows per codec side. In these executions the chronology resolves to `90` private pre-roll frames, `300` authentic application frames, & `5` post-roll / drain frames. Those private counts are observations of the readiness / terminal handshake rather than application constants; native ".csv" `frame_id` attribution remains authoritative for the 300 real frames.

Accordingly, `vstats` rows must not be naively joined by ordinal position to `telemetry_encoder.csv` or `telemetry_decoder.csv`. "codec"-level rate / frame counters complement the native telemetry but occupy a distinct measurement domain.

### 20.3 Timing Quantities Must Not Be Added Indiscriminately

Per-node timings intentionally overlap. `active_process_ms`, `total_residency_ms`, "codec" service time, "GPU" sub-stages, source-to-node latency, & browser "CTP" answer different questions. Adding them as though they were independent serial stages would double-count concurrent / nested work.

The most appropriate native end-to-end frontier is the `camera_send_timestamp` propagated by `Camera` & resolved by each terminal node. Its boundary is mode-specific by construction: for the principal `MIDDLE + WARM` condition it is sampled after timed file acquisition but before coordinate serialisation, whereas `Camera.total_residency_ms` begins earlier & therefore retains source acquisition. This distinction must be preserved when comparing source-local work against downstream `Camera`-origin latency. Browser `cmd_photon_ms` is further distinct because it terminates at the browser-side render-completion proxy described in Section 12.6.

### 20.4 Quality Columns Are Mode-Dependent

`mse_y`, `psnr_y`, & `ssim_y` are populated only when `Encoder` quality capture is active. Likewise `mean_error`, `geom_rmse`, `chamfer`, `hausdorff`, & their millimetre forms are merged by `Gauge` after a quality run. Interactive runtime telemetry intentionally leaves these fidelity fields unset.

### 20.5 Native Telemetry Serialization & Measurement Isolation

Native per-frame telemetry is accumulated within bounded in-memory structures throughout the active stream & is serialised to the corresponding `.csv` artefacts only after the terminal condition has been resolved. Consequently, native ".csv" file I / O is excluded from the 300-frame real-time measurement path. The quality mechanisms preserve the identical principle: `Encoder` retains "luma" references / attributed compressed material in memory, while `User` retains complete reconstructed snapshots in `quality_buffer`; persistent quality artefacts & objective evaluation are deferred until after "EOS".

The visual path follows the same rule. With `DEBUG_VISUALS_ENABLED`, Encoder & Decoder preserve only frame `195` through a preallocated full-`I420` snapshot; `.i420` / `.pgm` serialization occurs after stream finalisation. The remaining one-shot memory copy is intentionally not hidden behind another thread: all logical cores already participate in the validated deployment, & an asynchronous writer would move scheduler / cache / memory pressure rather than eliminate it. Visual runs are therefore diagnostic archives, whereas clean performance runs use `DEBUG_VISUALS_DISABLED`.

The `ffmpeg.txt` files constitute a deliberate exception in **measurement domain**, not in ".csv" semantics. They are "FFmpeg"-originated diagnostic chronologies associated with the persistent "codec" processes & remain analytically separate from the authoritative native per-frame ".csv" files. They must therefore neither be interpreted as application-frame tables nor be used to redefine the native timing frontiers documented above.

---

## 🧪 21. Relevant Outcomes from the Validated Snapshot

Validation evidence is separated by measurement purpose. The capture-enabled archive supplies sequence-level fidelity metrics, whereas Section 21.16 uses three executions of the common `B0` runtime configuration as the numerical reference for the completed non-capture sensitivity sweep. Browser command traces were not replayed identically, so primary-route timing & integrity are assessed for run-to-run stability while "Command-to-Photon" observations remain trace-specific. Variants that change cadence, temporal admission, or terminal completeness are interpreted as different operating points rather than folded into the same latency ranking. Earlier archives remain tied to the scope stated in their respective subsections & are not pooled with `B0`.

### 21.1 Dataset & Streaming Population

The repeated runtime baseline, `BQ`, & `OQ-OFF` operate upon the same 300-frame "Loot" binary sequence containing `238,146,391` original points. Every `B0` repetition reconstructs the identical final `valid_points` vector, for a sequence total of `43,620,972` points. The two capture-enabled controls likewise converge to that same frame-by-frame final population.

The quality-side intermediate populations are:

| Quantity | `BQ` | `OQ-OFF` |
|---|---:|---:|
| Original points | `238,146,391` | `238,146,391` |
| Arrived reconstruction candidates | `49,516,082` | `49,521,434` |
| Eroded points | `46,555,160` | `46,556,158` |
| Valid reconstructed points | `43,620,972` | `43,620,972` |

The intermediate `arrived_points` / `eroded_points` populations are not bit-identical between the two quality executions, whereas `valid_points` is identical for every one of the 300 corresponding frame IDs. The strict final population therefore remains invariant even when the complete geometric frontier is recomputed locally at `Encoder` rather than consumed from `SFF1`.

### 21.2 Complete-Chain Integrity

The consolidated reference / fidelity evidence is:

| Node / Route | Repeated Runtime `B0` | `BQ` | `OQ-OFF` |
|---|---|---|---|
| `Camera` | `900 / 900` admitted / Tx | `300 / 300` admitted / Tx | `300 / 300` admitted / Tx |
| `SFF1` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `SFF2 Route 0` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `Encoder` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `SFF2 Route 1` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `Decoder` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `SFF2 Route 2` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `SFF3` | `900 / 900` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` | `300 / 300` Rx / Tx, `100.0 %` |
| `User` | `900 / 900` Rx complete, `100.0 %` | `300 / 300` Rx complete, `100.0 %` | `300 / 300` Rx complete, `100.0 %` |

All primary protocol-error counters are zero in those executions. `Encoder` reports no write `EAGAIN` & `Decoder` reports no write failure / codec-queue drop; application `mbuf` starvation remains zero. Runtime conditions that intentionally alter temporal admission or terminal completeness are treated separately in Section 21.16 rather than being merged into this lossless reference set.

### 21.3 Camera — Nominal 30 Frames / s Source Operation & Local Backpressure

The steady-state `10 -> 285` source span remains close to the nominal 30-fps objective in every complete reference / fidelity execution:

| Execution | Measured Source Cadence | `current_skip` | Complete Source Frames |
|---|---:|---:|---:|
| `B0-R1` | `30.0027 frames / s` | `1` | `300` |
| `B0-R2` | `29.9998 frames / s` | `1` | `300` |
| `B0-R3` | `29.9958 frames / s` | `1` | `300` |
| `BQ` | `30.0011 frames / s` | `1` | `300` |
| `OQ-OFF` | `30.0020 frames / s` | `1` | `300` |

The direct span measurement is preferred to inverting one noisy per-frame interval because it preserves the complete observed time base. `BQ` reports steady medians of `2.888 ms` for `disk_io_ms`, `2.220 ms` for serialisation, `7.712 ms` for Tx duration, & `12.889 ms` for `active_process_ms`. `OQ-OFF` records `2.921`, `2.248`, `7.100`, & `12.499 ms`, respectively. Those source-local differences are host-phase observations rather than consequences of geometry placement, because the fair quality ablation changes the downstream / in-path geometry split rather than the `Camera` implementation.

The capture-enabled executions preserve identical 300-frame completion even though their local `Camera` Tx-acceptance phases differ materially:

| Condition | Frames with Zero Accepts | Zero-Accept Sum | Re-Presented Packets | Partial Accepts | `mbuf` Starvation |
|---|---:|---:|---:|---:|---:|
| `BQ` | `300` | `549,842` | `17,391,095` | `0` | `0` |
| `OQ-OFF` | `0` | `0` | `0` | `0` | `0` |

These counters describe repeated local `rte_eth_tx_burst()` presentation attempts. They are **not "UDP" retransmissions**. `BQ` experiences substantial local Camera-side acceptance pressure, whereas the final fair `OQ-OFF` capture run records none; both nevertheless deliver all 300 source frames completely, so the counter difference is treated as a host / queue-phase observation rather than an application-integrity difference.

#### 21.3.1 30 Frames / s Cadence Across the Complete `BQ` Route

Absolute `recv_start_timestamp` / source timestamps over frames `10 -> 285` establish the following rate continuity:

| Observation Frontier | Measured Rate | Mean Raw Interval |
|---|---:|---:|
| `Camera` departure | `30.001 frames / s` | `33.331 ms` |
| `SFF1` input | `30.002 frames / s` | `33.331 ms` |
| `SFF2 Route 0` input | `30.002 frames / s` | `33.331 ms` |
| `Encoder` input | `30.002 frames / s` | `33.331 ms` |
| `SFF2 Route 1` input | `30.012 frames / s` | `33.317 ms` |
| `Decoder` input | `30.012 frames / s` | `33.317 ms` |
| `SFF2 Route 2` input | `30.000 frames / s` | `33.330 ms` |
| `SFF3` input | `30.000 frames / s` | `33.330 ms` |
| `User` input | `30.000 frames / s` | `33.330 ms` |

All 300 frames remain complete at `current_skip = 1`. `OQ-OFF` independently sustains the same source objective without invoking "Temporal", while the three browser-enabled `B0` repetitions provide the runtime counterpart of the same nominal rate.

### 21.4 SFF1 / "GAC" — In-Path Geometry Cost

The updated capture-enabled `OQ-OFF` run now follows the same fair placement rule as the runtime `O-OFF` pair: `SFF1.NETWORK_PROCESSING` is disabled while the forwarding / classification / service-envelope role remains active. The steady medians therefore expose the placement change directly:

| `SFF1` Metric | Campaign `B0` | `BQ` | `OQ-OFF` |
|---|---:|---:|---:|
| `geometry_aggregation_ms` | `6.921 ms` | `7.152 ms` | `0.000 ms` |
| `max_r_ms` | `2.732 ms` | `3.694 ms` | `0.000 ms` |
| `active_process_ms` | `12.366 ms` | `13.199 ms` | `1.674 ms` |

`BQ` therefore measures the complete in-path "GAC" frontier, whereas `OQ-OFF` retains only `SFF1`'s forwarding / service-chain responsibilities & reconstructs the equivalent geometry once at `Encoder`. The quality pair now controls both semantic equivalence & placement without duplicated geometry work.

### 21.5 SFF2 — Three Validated Proxy Transitions

Steady median active costs remain small relative to the application / geometry frontiers:

| Route | Campaign `B0` | `BQ` | `OQ-OFF` |
|---|---:|---:|---:|
| `SFF2 Route 0` | `6.179 ms` | `4.530 ms` | `4.393 ms` |
| `SFF2 Route 1` | `0.009 ms` | `0.008 ms` | `0.008 ms` |
| `SFF2 Route 2` | `0.620 ms` | `0.540 ms` | `0.546 ms` |

Route 1 remains predominantly a compressed-media relay, Route 2 re-encapsulates reconstructed points, & Route 0 additionally handles the geometry-bearing application context. All three preserve complete quality-control populations, so the small differences between `BQ` & `OQ-OFF` are treated as execution-phase variation rather than an offload-path semantic effect.

### 21.6 Encoder — Geometry Offload, "GPU", Workload Control, & "H.265"

The repeated runtime reference is best represented by the median of the three steady run-level medians:

| Encoder Metric | Campaign `B0` |
|---|---:|
| `conversion_ms` | `4.189 ms` |
| `projection_ms` | `5.421 ms` |
| `codec_write_ms` | `8.018 ms` |
| `encode_service_ms` | `32.325 ms` |
| `encode_h265_ms` | `32.326 ms` |
| `active_process_ms` | `17.684 ms` |
| `workload_ratio` | `0.166` |

Complete offloaded frames retain `geometry_aggregation_ms = 0` & `max_r_ms = 0` at `Encoder`. `B0-R3` nevertheless demonstrates that local writer / codec phase is not perfectly repeatable: its `codec_write_ms` median reaches `30.763 ms` & `encode_h265_ms` reaches `49.130 ms` while `frame_backlog = 0`, `codec_backlog <= 2`, write `EAGAIN = 0`, & the workload ratio remains below the overload frontier. The absence of matched "Pose" commands in `B0-R3` excludes browser interaction as a sufficient explanation for that shift.

The capture-enabled controls now provide a fair quality-side placement comparison. `BQ` reports `Encoder.active_process_ms = 24.140 ms`, `codec_write_ms = 15.508 ms`, `encode_h265_ms = 39.507 ms`, & `workload_ratio = 0.167`, with zero local geometry work at `Encoder`. In `OQ-OFF`, `SFF1` geometry is disabled & `Encoder` pays `6.039 ms` of geometry aggregation / transformed-frontier work plus `3.348 ms` of `max_r`; `Encoder.active_process_ms` rises to `37.493 ms`, `codec_write_ms` to `18.043 ms`, `encode_h265_ms` to `41.054 ms`, & the workload-ratio median to `0.465` ( maximum `0.499` ). No "Temporal" transition occurs, so the quality run confirms the same placement effect while preserving the complete 300-frame operating point.

A three-frame Route-1 modulation is visible in one of the repeated runtime traces but is not campaign-wide:

| Run | Rate | `inter_arrival_ms` Std. Dev. | Lag-3 Correlation | `% 3 = 0 / 1 / 2` Mean ( ms ) |
|---|---:|---:|---:|---:|
| `B0-R1` | `29.987 frames / s` | `2.857` | `0.143` | `33.171 / 33.252 / 33.613` |
| `B0-R2` | `29.980 frames / s` | `2.903` | `0.198` | `33.232 / 33.598 / 33.253` |
| `B0-R3` | `29.964 frames / s` | `4.925` | `0.971` | `33.290 / 27.458 / 39.323` |
| `BQ` | `30.012 frames / s` | `0.730` | `-0.041` | `32.980 / 33.427 / 33.544` |

`B0-R3` therefore contains a strong local lag-3 output phase, but `R1`, `R2`, & `BQ` do not reproduce it. Its long-window Route-1 mean remains approximately one 30-fps period & all frames remain complete. The evidence supports an execution-phase / persistent-codec scheduling effect rather than a fixed three-frame protocol rule.

### 21.7 Decoder — Hardware Decode & Reconstruction

The Decoder remains close to one-frame service cadence even when its broader pipelined latency frontier varies:

| Decoder Metric | Campaign `B0` | `BQ` | `OQ-OFF` |
|---|---:|---:|---:|
| `reconstruction_pipeline_ms` | `2.470 ms` | `2.713 ms` | `2.393 ms` |
| `active_process_ms` | `37.655 ms` | `37.409 ms` | `36.922 ms` |
| `decode_service_ms` | `33.414 ms` | `33.283 ms` | `33.239 ms` |
| `decode_h265_ms` | `128.881 ms` | `124.078 ms` | `123.886 ms` |
| Camera -> `Decoder` `e2e_latency_ms` | `196.904 ms` | `196.729 ms` | `196.670 ms` |

`decode_service_ms` is the more direct service-rate indicator; `decode_h265_ms` is a pipelined latency frontier with several frames concurrently resident in the persistent hardware path & must not be interpreted as inverse decode throughput. No "codec" queue drop, "FFmpeg" write failure, `mbuf` starvation, or downstream frame loss is observed in `B0`, `BQ`, or `OQ-OFF`.

### 21.8 SFF3 & User Terminal Delivery

The two complete quality controls remain lossless at the final native boundary:

| Terminal Metric | `BQ` | `OQ-OFF` |
|---|---:|---:|
| `SFF3.active_process_ms` steady median | `0.734 ms` | `0.746 ms` |
| `User.active_process_ms` steady median | `0.645 ms` | `0.668 ms` |
| `User.reference_e2e_ms` all-frame mean | `197.250 ms` | `201.140 ms` |
| `User.reference_e2e_ms` all-frame median | `196.944 ms` | `197.124 ms` |
| `User.reference_e2e_ms` steady median | `196.820 ms` | `196.981 ms` |
| `User.reference_e2e_ms` all-frame P95 | `199.065 ms` | `226.341 ms` |

Both runs receive 300 / 300 complete native frames, & `SFF3` reports zero Tx zero-accept / partial-accept pressure in the two quality conditions. `OQ-OFF` therefore preserves terminal completeness after relocating geometry to `Encoder`, but its larger all-frame mean / P95 exposes a heavier tail in this capture-enabled execution. The quality pair is used primarily for semantic / fidelity validation; browser-enabled `B0-R1` remains the runtime comparator for performance ranking.

### 21.9 Interactive "Pose" / "Command-to-Photon" Results

The interaction stream was not replayed identically among runtime repetitions. `B0-R1` contains the richest trace, with `52` command identifiers matched to returning pose states; `B0-R2` contains `5`, & `B0-R3` contains none. `B0-R1` records `95` positive browser frame acknowledgments, a count that describes rendered snapshots rather than browser frame rate or command count.

The `B0-R1` timing populations are:

| Frontier | Samples | Mean ( ms ) | Median ( ms ) | P95 ( ms ) | Max ( ms ) |
|---|---:|---:|---:|---:|---:|
| `reference_cmd_ms` | `52` | `14.857` | `10.276` | `37.904` | `47.311` |
| `cmd_apply_ms` | `52` | `33.488` | `33.928` | `58.667` | `63.625` |
| `Decoder.pose_control_ms` | `52` | `18.468` | `19.398` | `34.428` | `44.261` |
| `cmd_photon_ms` | `51` positive samples | `184.196` | `178.000` | `265.500` | `302.000` |

All 52 requests are observed in the returning native pose state; one does not acquire a positive browser `cmd_photon_ms` sample before experiment closure. Across all 300 `B0-R1` rows, command-associated `reference_e2e_ms` has a median of `213.851 ms`, versus `210.507 ms` for the remaining rows ( approximately `+3.345 ms` ). This association is retained as a trace-specific nuisance-variable observation rather than as a causal explanation for cross-run codec differences.

### 21.10 Objective Encoder Quality

Both 300-frame capture-enabled runs produce complete "luma" indicators:

| Metric | `BQ` Mean | `BQ` Median | `BQ` P5 | `BQ` P95 | `OQ-OFF` Mean | `OQ-OFF` Median | `OQ-OFF` P5 | `OQ-OFF` P95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `MSE-Y` | `0.241633` | `0.220` | `0.200` | `0.320` | `0.241833` | `0.220` | `0.200` | `0.321` |
| `PSNR-Y` ( dB ) | `54.3579` | `54.660` | `53.030` | `55.120` | `54.3557` | `54.670` | `53.020` | `55.110` |
| `SSIM-Y` | `0.997368` | `0.998205` | `0.991364` | `0.998609` | `0.997359` | `0.998201` | `0.991175` | `0.998597` |

Because larger "PSNR" / "SSIM" values indicate better fidelity, the added `P5` values expose their adverse lower tail while `P95` remains visible for distribution completeness. `MSE-Y` retains both tails under the same tabular convention.

The aggregate fair-placement differences remain extremely small: `OQ-OFF - BQ` is approximately `+0.000200` "MSE", `-0.0022 dB` "PSNR", & `-8.06e-6` "SSIM". They do not establish bit-identical encoded pictures; rather, **no material systematic fidelity shift is observed at the reported sequence-level resolution when the same geometric frontier is relocated from `SFF1` to the aligned local `Encoder` path**. These values describe the custom projected "luma" representation, not a standards-compliant point-cloud "codec" rate-distortion curve.

The common 300 `frame_id` values also permit a direct paired comparison instead of relying solely upon aggregate summaries. Defining `Delta = OQ-OFF - BQ` for each aligned frame gives:

| Paired "luma" Metric | Median `Delta` | P95 `abs( Delta )` | Maximum `abs( Delta )` |
|---|---:|---:|---:|
| `MSE-Y` | `0.000` | `0.010` | `0.010` |
| `PSNR-Y` | `0.000 dB` | `0.061 dB` | `0.180 dB` |
| `SSIM-Y` | `-4.50e-6` | `1.31e-4` | `2.49e-4` |

The paired view reinforces the aggregate observation: frame-aligned "luma" deviations remain extremely small, while the nonzero maxima explicitly prevent the result from being misrepresented as byte-identical encoded output.

### 21.11 Objective Reconstructed Geometry Quality

`Gauge` evaluates all 300 complete frames in both quality controls. Its millimetre indicators are **post-registration shape-fidelity quantities**: inverse-pose normalisation, statistical filtering, & robust "ICP" precede nearest-neighbour evaluation, so these values do not independently quantify residual global pose / rigid-registration error. The telemetry field `mean_error` is consequently named in the prose below as **directed reconstructed -> reference mean error** rather than as an unqualified geometric mean:

| Metric | `BQ` Mean | `OQ-OFF` Mean | Relative Difference | Unit |
|---|---:|---:|---:|---|
| Directed reconstructed -> reference mean error | `6.3859` | `6.3894` | `+0.054 %` | `mm` |
| Geometric "RMSE" | `6.2590` | `6.2556` | `-0.054 %` | `mm` |
| Symmetric Chamfer | `11.1645` | `11.1680` | `+0.031 %` | `mm` |
| Symmetric Hausdorff | `121.1533` | `121.0110` | `-0.117 %` | `mm` |

The corresponding `BQ` medians are `6.145 / 6.142 / 10.922 / 105.444 mm`; `OQ-OFF` records `6.148 / 6.122 / 10.946 / 106.122 mm`. The Hausdorff metric retains the expected larger tail & must be interpreted together with the directed mean / "RMSE" / Chamfer measures rather than in isolation.

The same aligned 300-frame population permits a paired geometric comparison with `Delta = OQ-OFF - BQ`:

| Paired Post-"ICP" Geometry Metric | Median `Delta` | P95 `abs( Delta )` | Maximum `abs( Delta )` |
|---|---:|---:|---:|
| Directed reconstructed -> reference mean error | `+0.004 mm` | `0.270 mm` | `0.425 mm` |
| Geometric "RMSE" | `+0.003 mm` | `0.231 mm` | `0.327 mm` |
| Symmetric Chamfer | `+0.006 mm` | `0.394 mm` | `0.666 mm` |
| Symmetric Hausdorff | `-0.157 mm` | `35.455 mm` | `227.316 mm` |

The paired directed-mean / "RMSE" / Chamfer deviations remain tightly concentrated. Hausdorff, as an extreme-value metric, exhibits substantially wider frame-level excursions despite its nearly unchanged sequence mean; this tail behaviour is therefore retained explicitly rather than being hidden behind the aggregate `-0.117 %` difference.

The equality of `valid_points` for every aligned frame, together with nearly unchanged "luma" & post-`Gauge` aggregate metrics, completes the fair capture-enabled geometry-placement control. The result supports the direct statement that **relocating the complete geometric frontier from `SFF1` to equivalent local `Encoder` computation does not produce a systematic material fidelity shift in the measured implementation**; it does not imply frame-wise identity of every extreme-value metric or byte-for-byte identity of intermediate atlas / compressed-stream representations.

### 21.12 Quality-Capture Runtime Cost

The stable `User` capture path retains the reconstructed frame in the ordinary `web_points` region & copies only complete frames into the 1-GiB capture buffer. The measured distributions are:

| `quality_save_ms` | Mean | Median | P95 | P99 | Max |
|---|---:|---:|---:|---:|---:|
| `BQ` | `2.234 ms` | `2.187 ms` | `2.338 ms` | `4.078 ms` | `6.147 ms` |
| `OQ-OFF` | `3.730 ms` | `2.303 ms` | `14.006 ms` | `17.903 ms` | `32.312 ms` |

Even the larger `OQ-OFF` maximum remains below the nominal `33.333 ms` source period, & neither quality run records `SFF3` Tx pressure or incomplete terminal frames. Objective `Gauge` computation begins only after "EOS" & is excluded from stream-time values.

`quality_save_ms` remains an independent instrumentation cost rather than being folded into `User` `node_exit_timestamp`, `e2e_latency_ms`, `total_residency_ms`, or `active_process_ms`. The native frame-ready frontier remains the terminal timing reference, while the subsequent complete-frame memory copy is exposed separately.

#### 21.12.1 Deferred Frame-195 Visual Validation

The dedicated diagnostic execution captures frame `195` on both sides of the live "H.265" path without introducing persistent file I / O into the timed stream. The supplied `res_debug` archive contains exactly eight artefacts: one complete `I420` image plus `Geometry`, `Texture-Y`, & `Occupancy` grayscale planes from `Encoder`, together with the corresponding decoded outputs. Each `.pgm` is `2560 x 1536`; the full `I420` representation is `2560 x 4608`, occupying `17,694,720 B`.

A direct byte-level comparison of the supplied artefacts gives:

| Comparison | "MAE" | "PSNR" | Exact Samples | Maximum Difference |
|---|---:|---:|---:|---:|
| Complete `I420` | `0.0397` | `56.08 dB` | `98.29 %` | `25` |
| `Geometry` | `0.0334` | `58.81 dB` | `97.84 %` | `25` |
| `Texture-Y` | `0.1089` | `50.85 dB` | `96.52 %` | `22` |
| `Occupancy` | `0.0098` | `63.32 dB` | `99.50 %` | `13` |

For the complete raw picture, the component-level comparison is:

| `I420` Component | "MAE" | "PSNR" | Exact Samples | Maximum Difference |
|---|---:|---:|---:|---:|
| `Y` | `0.0507` | `54.77 dB` | `97.95 %` | `25` |
| `U` | `0.0180` | `61.40 dB` | `98.94 %` | `21` |
| `V` | `0.0172` | `61.38 dB` | `98.99 %` | `24` |

The `Complete I420` figure aggregates all raw `Y`, `U`, & `V` samples & is therefore a frame-specific diagnostic quantity; it must not be substituted for the sequence-level `PSNR-Y` reported in Section 21.10. Likewise, the exact-sample percentage is descriptive rather than a stand-alone quality metric because the atlas contains a large zero-valued background.

Visual inspection confirms the same six-view arrangement before & after coding: the object silhouettes retain their positions, view ordering, & gross geometry, with residual differences concentrated around contours & textured regions rather than manifesting as missing views or large spatial displacements. This qualitative observation is consistent with the low-amplitude pixel errors above, while the quantitative support test provides the stronger topology check.

`Encoder` Occupancy contains `148,516` active pixels. Applying a threshold greater than `16` to the decoded plane recovers exactly the same `148,516`-pixel binary support, yielding `IoU = 1.0`, precision `= 1.0`, & recall `= 1.0`. Non-zero values below the threshold correspond to low-amplitude codec ringing rather than an occupancy-topology change.

This execution is retained strictly as a visual / representation validation. Serialization occurs only after stream finalisation, yet keeping frame `195` requires one `17,694,720 B` memory copy at each diagnostic node. Performance measurements therefore use the debug-disabled path; the visual evidence is reported separately rather than mixed into the configuration campaign.

### 21.13 Active-Processing Cross-Condition View

The steady-state median `active_process_ms` values provide a compact non-additive view of where the updated placement work is paid. `Camera.active_process_ms` remains the historical source aggregate defined in Section 6.6 — acquisition + serialisation + Tx wall-clock duration — whereas downstream nodes generally use active-work definitions specific to their implementation. The column is therefore used for **local attribution**, not as a sum of host-"CPU" time across the route.

| Node / Route | `B0-R1` | `O-OFF-R1` | `O-OFF-R2` | Placement reading |
|---|---:|---:|---:|---|
| `Camera` | `18.068 ms` | `12.661 ms` | `16.575 ms` | source-side variation, not offload location |
| `SFF1` | `12.366 ms` | `1.681 ms` | `2.464 ms` | in-path geometry is disabled in fair `O-OFF` |
| `SFF2 Route 0` | `6.196 ms` | `3.861 ms` | `6.621 ms` | proxy cost remains route-local |
| `Encoder` | `17.684 ms` | `54.335 ms` | `26.081 ms` | local geometry reconstruction consumes Encoder headroom |
| `SFF2 Route 1` | `0.009 ms` | `0.009 ms` | `0.008 ms` | compressed-media proxy remains negligible |
| `Decoder` | `37.636 ms` | `36.240 ms` | `37.729 ms` | downstream decode frontier remains comparable |
| `SFF2 Route 2` | `0.618 ms` | `0.637 ms` | `0.624 ms` | reconstructed-point proxy remains stable |
| `SFF3` | `1.057 ms` | `0.852 ms` | `1.092 ms` | final aware boundary remains light outside `P40` |
| `User` | `2.332 ms` | `0.715 ms` | `2.167 ms` | terminal publication / interaction phase is trace-dependent |

These values must not be summed into an end-to-end latency estimate because stages overlap or represent nested asynchronous work. The placement result is nevertheless clear: when `SFF1.NETWORK_PROCESSING` is disabled, `SFF1` stops paying the geometric cost & the equivalent frontier appears at `Encoder`. The two fair attempts differ in writer / codec phase, but both raise the steady Encoder workload-ratio median above the in-network baseline ( `0.419 / 0.391` versus `0.165` ).

### 21.14 "FFmpeg" Cross-Check

Every complete 300-frame runtime / fidelity / placement run — including the two fair `O-OFF` attempts — contains `395` `vstats` rows per Encoder / Decoder side: `90` private pre-roll outputs, `300` application frames, & `5` terminal post-roll outputs. Active `P40` contains `365 = 90 + 270 + 5` rows because the workload controller admits `270` source frames, while the no-`Temporal` stress check returns to `395` rows. These codec chronologies independently agree with source admission.

All archived Encoder / Decoder `stderr.log` files are empty. Private warm-up / drain rows remain handshake chronology rather than application-frame evidence; native ".csv" completion & `frame_id` attribution remain authoritative.

### 21.15 Direct Comparison with the Application-Level Reference Baseline

A direct comparison was performed against the supplied 300-frame **probe-disabled "GPU" reference logs** associated with the Lacaria pipeline & against the 10-Mbit / s quality values reported in the reference thesis. Only frontiers with sufficiently compatible semantics are compared. The resulting number is treated as the **system-level gain of the proposed architecture**: native "DPDK" gives the complete graph one uniform packet-processing substrate, "SFC" makes steering / reverse control explicit, & the principal computational mechanism introduced into the forwarding path is `SFF1` in-network geometry. The present `Camera`-origin frontier is anchored by `camera_send_timestamp`, which in `MIDDLE + WARM` is sampled after timed file acquisition & before serialisation; the comparison therefore keeps semantically compatible frame-ready boundaries rather than requiring cycle-identical source internals. The two browser-enabled fair `O-OFF` reruns are the dedicated runtime ablation used to quantify the headroom / latency cost of removing that in-network frontier, while fair capture-enabled `OQ-OFF` verifies the corresponding fidelity behaviour.

For runtime responsiveness, the repeated `B0` set in Section 21.16 provides the preferred current-system evidence. Across all 300 frames, the three native Camera-to-User medians are:

| Execution | Median Frame-Ready Latency | Reduction vs. Reference |
|---|---:|---:|
| `B0-R1` | `211.352 ms` | `38.16 %` |
| `B0-R2` | `214.176 ms` | `37.34 %` |
| `B0-R3` | `218.310 ms` | `36.13 %` |

The best complete runtime trace for cross-condition comparison is therefore `B0-R1` at `211.352 ms`, corresponding to a `38.16 %` reduction against the supplied `341.785 ms` reference frontier. The median of the three run-level medians remains `214.176 ms`; its `6.958 ms` range & approximately `1.63 %` run-level coefficient of variation document host / codec-phase dispersion without changing the headline best-run comparison requested for the updated campaign.

This comparison terminates at complete native frame availability, not at browser rendering. `BQ` reaches an all-frame median of `196.944 ms`, but it is not used as the headline performance number because quality mode removes browser publication & adds capture instrumentation. The triplicated runtime comparator remains documented, while the slide-level comparisons use the best complete runtime trace (`B0-R1`) consistently.

Selected application frontiers show the same direction when the updated comparison is anchored to `B0-R1` while retaining the repeated set as variability evidence:

| Semantically Related Frontier | Reference Median | `B0-R1` / repeated context | Relative Change | Interpretation |
|---|---:|---:|---:|---|
| Encoder point conversion | `8.667 ms` | `4.189 ms` | `-51.7 %` | lower Encoder-local conversion cost in the decomposed path |
| Encoder projection | `10.882 ms` | `5.421 ms` | `-50.2 %` | shorter projection frontier while upstream "GAC" work remains charged separately |
| Post-decode unpack / erosion / reconstruction-render work | `6.108 ms` | `2.470 ms` | `-59.6 %` | semantically related reconstruction frontier rather than a bit-identical implementation |

These values show that the gain is produced by the **combined data-plane design rather than by removing work from measurement**. The proposed graph adds measurable forwarding & geometric work, yet the repeated complete system preserves the 300-frame / nominal-30-fps objective while lowering compatible terminal latency & selected application-stage medians. "DPDK" supplies the uniform execution model, "SFC" supplies the steering / control contract, & the in-network geometry stage removes reusable frame-preparation work from the `Encoder` boundary. `B0-R3` also motivates reporting codec-handoff values separately, so that the in-network contribution is established by the dedicated fair geometry ablation rather than by codec-phase fluctuations.

#### Fidelity Relative to the Reference

The decomposition preserves the same source sequence exactly at the population level. The mean source population is `793,821.303` points / frame in both data sets. More significantly, the final valid reconstructed population is almost identical:

```text
Reference mean valid_points = 145,403.233
Current mean valid_points   = 145,403.240
Difference                  = 0.007 points / frame
```

Across the full 300-frame sequence, `BQ` reconstructs `43,620,972` valid points, only about `2` aggregate points above the reference total implied by `145,403.233` points / frame. The mean final-population ratio is approximately `18.317 %` of the original cloud in both implementations. This is strong evidence that service decomposition, additional `SFF` traversal, & reference-compatible offloading do **not** introduce systematic reconstruction-population loss.

Signal & geometric quality must be stated more carefully. At the common 10-Mbit / s condition, the reference thesis reports approximately `MSE-Y = 0.129`, `PSNR-Y = 56.06 dB`, & `SSIM-Y = 0.997`; `BQ` reports `0.241633`, `54.3579 dB`, & `0.997368`, respectively. Structural similarity therefore remains in the same high-fidelity regime, while `PSNR-Y` is approximately `1.70 dB` lower & `MSE-Y` is higher. Exact "luma"-fidelity invariance relative to the reference implementation is **not** claimed.

An additional comparability constraint originates from the execution environment itself. The current "CUDA" kernels are compiled explicitly for `sm_61`, while the host-side build additionally enables `-ffast-math`; the resulting numerical path is tied to a specific compilation target rather than being guaranteed to reproduce bit-identical floating-point operations across heterogeneous "GPU" architectures. At the same time, the compressed representation is produced by the hardware "NVENC" path, whose concrete execution environment includes the "GPU" generation, NVIDIA driver, "FFmpeg" build, & encoder implementation available during the run.

The observed "PSNR-Y" / "MSE-Y" difference must consequently not be interpreted as a distortion introduced by "SFC" or "UDP" alone. The present comparison changes several variables simultaneously, including service decomposition, "CUDA" compilation / execution conditions, persistent `codec` handling, & the hardware-software video stack. Establishing causal attribution for the residual quality difference would require rebuilding & executing both architectures under an identical "GPU", driver, "CUDA" toolchain, "FFmpeg" / "NVENC" configuration, source representation, & metric implementation.

The same caution applies to robust post-erosion geometry. Converting the reference thesis's 10-Mbit / s robust values through the common `1 voxel = 1.820 mm` scale gives:

| Metric | Reference Robust Post-Erosion | Current `BQ` Gauge | Difference in Interpretation |
|---|---:|---:|---|
| Directed reconstructed -> reference mean error | `4.472 mm` | `6.386 mm` | Current value is higher; both remain millimetre-scale |
| Geometric "RMSE" | `5.586 mm` | `6.259 mm` | Same order of magnitude |
| Symmetric Chamfer | `10.025 mm` | `11.165 mm` | Same order of magnitude |
| Symmetric Hausdorff | `87.364 mm` | `121.153 mm` | Current worst-case tail is higher |

The cross-architecture evidence therefore supports **fidelity preservation at the same high-quality operating regime, not numerical identity**. The `BQ` / `OQ-OFF` control in Sections 21.10 & 21.11 now applies the same fair geometry placement used by the runtime ablation & shows that the relocated local `Encoder` frontier preserves objective fidelity, while the browser-enabled `O-OFF-R1 / R2` pair quantifies the corresponding runtime headroom / latency cost.

#### Orchestration & Request-Handling Consequences

The reference thesis explicitly identifies its monolithic `Encoder` & Client placement as an orchestration limitation & proposes splitting projection / encoding & decoding / client processing into independently deployable blocks. The current architecture directly advances that objective by exposing seven native graph-level placement units instead of four coarse services. This is a `75 %` increase in **placement granularity**, not a 75 % resource saving.

The control model also removes dedicated transport-connection state from the native service plane. The reference architecture maintains two persistent "TCP" feedback channels, each bound to a specific upstream application node. The current design instead classifies two fixed-size control datagrams into independent `SPI 200` & `SPI 300` service paths. A new request family can consequently be represented through a new classifier / service-path contract without requiring `Encoder` / `Decoder` to acquire "SFC" parsing logic or introducing another byte-stream framing scheme into the `SFF` path.

This improves **orchestration flexibility**, but present evidence does not establish a maximum simultaneous-user or request-rate gain. The reference thesis itself notes that its proposed Density Test would replicate the complete four-node stack because the original application has no stream identifier & accepts one connection per socket. The current implementation likewise remains a single-stream experimental chain; it has not executed an N-user density test. Its demonstrated advantage is finer placement / steering granularity & O( 1 )-sized latest-state control, while quantitative multi-stream capacity remains future work rather than a claimed result.

#### Measured Result Summary

The validated campaign supports five direct conclusions: ( i ) the reference `Encoder` / Client blocks are decomposed into network-visible functions; ( ii ) the best complete `B0` runtime trace preserves the 300-frame primary route at nominal source cadence; ( iii ) its Camera-to-User frame-ready latency is `211.352 ms` versus `341.785 ms` for Camera-to-Client reception in the supplied reference trace, an observed `38.16 %` reduction; ( iv ) the fair `SFF1.NETWORK_PROCESSING_DISABLED + Encoder.OFFLOAD_MODE_DISABLED` reruns show that removing in-network geometry raises `Encoder.active_process_ms` from `17.684 ms` to `54.335 / 26.081 ms`, workload-ratio median from `0.165` to `0.419 / 0.391`, & steady terminal latency from `211.222 ms` to `235.305 / 216.298 ms`; & ( v ) the paired `BQ` / `OQ-OFF` fidelity control keeps all 300 final `valid_points` populations frame-aligned while "PSNR-Y", "SSIM-Y", & `Gauge` aggregates remain materially unchanged at the measured resolution.

### 21.16 Configuration-Sensitivity Campaign & Repeated `B0`

The configuration campaign follows a **differential, one-factor-at-a-time** design. Deployment, affinity, queueing, scheduling, "codec", & service-path settings remain fixed while one independent condition changes. The objective is not to rank isolated numbers without context, but to identify which parts of the measured behaviour move beyond the empirical variability exposed by the repeated runtime condition.

The common configuration is:

```text
CACHE_MODE          = CACHE_MODE_MIDDLE
WARM_MODE           = WARM_MODE_ENABLED
OFFLOAD_MODE        = OFFLOAD_MODE_ENABLED
NETWORK_PROCESSING  = NETWORK_PROCESSING_ENABLED
TEMPORAL_ADAPTATION = TEMPORAL_ADAPTATION_ENABLED
TARGET_FPS          = 30
K_FRAMES            = 300
POINTS_PER_PACKET   = 80
NETWORK_MTU         = 1500
MEDIA_PAYLOAD_SIZE  = 1316
QUALITY_CAPTURE     = 0
```

`CACHE_MODE_MIDDLE + WARM_MODE_ENABLED` is retained for methodological rather than optimality reasons. `MIDDLE` preserves per-frame `fread()` within the measured source path, while warm mode controls page residency so that file acquisition does not collapse the nominal schedule. `BEST` moves source acquisition / serialisation outside the timed loop & is therefore an optimistic application-resident bound; `WORST` additionally introduces per-frame allocation / release & is retained as a pessimistic implementation bound. Neither replaces the principal scenario because each changes more of the source execution semantics than the warm `MIDDLE` condition required for the reference-like 30-fps operating point.

Changed fields are repeated below. `OQ-OFF` uses the same fair geometry-placement state as the runtime `O-OFF` pair while enabling quality capture, so runtime attribution & fidelity attribution now share one placement definition. Runtime sensitivity & the targeted capture-enabled control are complete for the supplied archive.

| Run | Change from Common Configuration | Experimental Question | Status |
|---|---|---|---|
| `B0-R1 / R2 / R3` | none | run-to-run stability of the complete runtime path | completed three times |
| `BQ` | `QUALITY_CAPTURE = 1` | objective fidelity at the common operating point | completed |
| `C-BEST` | `CACHE_MODE = BEST`, `WARM_MODE = N / A` | application-resident source bound | completed |
| `C-M-COLD` | `WARM_MODE = DISABLED` | effect of page residency with reusable staging | completed |
| `C-W-WARM` | `CACHE_MODE = WORST` | allocation-heavy source with resident pages | completed |
| `C-W-COLD` | `CACHE_MODE = WORST`, `WARM_MODE = DISABLED` | pessimistic cold-source bound | completed |
| `O-OFF-R1 / R2` | `SFF1.NETWORK_PROCESSING = DISABLED`, `Encoder.OFFLOAD_MODE = DISABLED` | fair placement ablation: in-network geometry removed & equivalent frontier computed once at `Encoder` | complete 300-frame runtime reruns; `Encoder` workload-ratio median `0.419 / 0.391` |
| `OQ-OFF` | `SFF1.NETWORK_PROCESSING = DISABLED`, `Encoder.OFFLOAD_MODE = DISABLED`, `QUALITY_CAPTURE = 1` | fair capture-enabled placement / fidelity control: geometry removed from `SFF1` & computed once at `Encoder` | completed; 300 / 300, no "Temporal" activation |
| `P40` | `POINTS_PER_PACKET = 40` | high point-datagram-rate stress | completed; "Temporal" activated, `270` admitted / `246` terminally complete |
| `P40-NO-T` | `POINTS_PER_PACKET = 40`, `TEMPORAL_ADAPTATION = DISABLED` | diagnostic isolation of the same packet-rate stress without source regulation | completed diagnostic; `300` admitted / `258` terminally complete |
| `P84` | `POINTS_PER_PACKET = 84` | near-`MTU` point aggregation | completed |
| `M1000` | `MEDIA_PAYLOAD_SIZE = 1000` | non-aligned compressed-media grouping | completed |

`C-BEST` has no independent warm/cold distinction because all source elements are already allocated, read, serialised, & `mlock()`-ed before streaming. `P84` is the largest whole-point capacity admitted by the 1500-byte IPv4 `MTU` once the complete `SFF1` service envelope is included; `85` points would exceed that bound. `M1000` changes application-level media grouping only: an "MPEG-TS" packet remains `188 B`, while the baseline `1316 B` grouping contains exactly seven transport packets.

For this campaign the packetisation constants were changed consistently across all binaries sharing the corresponding wire contract before each rebuild. `POINTS_PER_PACKET` therefore uses one common value across `Camera`, `SFF1`, `SFF2`, `Encoder`, `Decoder`, `SFF3`, & `User`; `MEDIA_PAYLOAD_SIZE` is shared by `Encoder`, `SFF2`, & `Decoder`. This avoids mixing runtime-advertised capacities with a different locally compiled fallback / startup constraint.

#### Archive Audit & Common Correctness Checks

The supplied archive contains `15` experiment folders used by the campaign: `13` browser-enabled runtime / diagnostic executions plus the two capture-enabled controls `BQ` & `OQ-OFF`. Every one contains the expected native ".csv" set together with `Encoder` / `Decoder` `ffmpeg.txt` & `stderr.log`. Every archived `stderr.log` is empty. All `SFF1`, `SFF2`, & `SFF3` protocol error counters remain zero in every run, so none of the observed differences is accompanied by an "Ethernet", "IPv4", "UDP", or service-header validation failure.

The nominal `B0`, source-residency, fair `O-OFF`, `P84`, & `M1000` runtime conditions preserve all 300 source frames through `User` at `current_skip = 1`. Active `P40` admits `270 / 300` source frames & reaches `User` with `246` complete frames because its separate packet-rate boundary stresses the `SFF3 -> User` hop. `BQ` & `OQ-OFF` each preserve 300 / 300 complete frames at `skip = 1`. Zero application / codec `mbuf` starvation is retained throughout the archive.

The "codec" chronologies independently match those application populations. Complete 300-frame executions, including both fair `O-OFF` attempts, contain `395` rows per Encoder / Decoder side ( `90` pre-roll + `300` application + `5` post-roll ); active `P40` contains `365 = 90 + 270 + 5`. This common audit is intentionally stated once; the subsections below describe only the behaviours that distinguish each condition.

#### Repetition Rule & Numerical Reference

Three complete executions of the unchanged runtime configuration are archived as `B0-R1`, `B0-R2`, & `B0-R3`. Frame-associated stage timings are reduced independently inside each run over the steady interval `10 -> 285`; the campaign comparator is the **median of the three run-level medians**. Their minimum / maximum range remains visible as the empirical stability envelope rather than being collapsed into the central value.

The all-frame `User.reference_e2e_ms` median-of-medians is `214.176 ms`. For the differential analysis below, the steady-state terminal comparator is `214.365 ms`, with a three-run envelope of `211.222 -> 217.313 ms`. This interval is retained explicitly because host / codec phase is part of the measured platform variability rather than noise that can be silently removed.

#### `B0` — Repeated Runtime Characterisation

The three executions preserve an identical source point sequence & complete primary-route semantics. In aggregate, `900 / 900` application-frame traversals complete at every native boundary, every available `data_integrity_pct` is `100.0 %`, & no protocol error is reported.

| Quantity | `B0-R1` | `B0-R2` | `B0-R3` | Cross-Run Observation |
|---|---:|---:|---:|---|
| Source points | `238,146,391` | `238,146,391` | `238,146,391` | identical |
| Source point datagrams | `2,976,979` | `2,976,979` | `2,976,979` | identical |
| Attributed "MPEG-TS" bytes | `10,346,956` | `10,346,392` | `10,354,664` | `8,272 B` span ( approximately `0.080 %` ) |
| Route-1 media datagrams | `7,990` | `7,992` | `8,002` | 12-packet span |
| Final valid points | `43,620,972` | `43,620,972` | `43,620,972` | identical frame by frame |
| Route-2 / User point datagrams | `545,406` | `545,406` | `545,406` | identical |

Intermediate `arrived_points` / `eroded_points` populations exhibit small run-level differences, but the strict `valid_points` vector converges to the same value for every corresponding frame before Route 2 transmission. The encoded-stream-size variation remains below one tenth of one percent of the approximately `10.35 MB` sequence & does not alter completion or final population.

##### Source Cadence & Local Tx Pressure

The source clock remains highly repeatable even though descriptor acceptance varies materially:

| Metric | `B0-R1` | `B0-R2` | `B0-R3` |
|---|---:|---:|---:|
| `Camera` cadence, frames / s | `30.0027` | `29.9998` | `29.9958` |
| `inter_departure_ms` median | `33.439` | `33.512` | `33.401` |
| `disk_io_ms` median | `3.239` | `3.309` | `4.328` |
| `serialization_ms` median | `2.484` | `2.542` | `2.837` |
| `tx_duration_ms` median | `11.955` | `11.929` | `7.485` |
| `active_tx_ms` median | `3.293` | `3.311` | `2.218` |
| `active_process_ms` median | `18.068` | `18.193` | `15.183` |

The span-derived source cadence has a run-level coefficient of variation of approximately `0.012 %` & a total range below `0.007 frames / s`. Source-side work is less repeatable than the absolute schedule because local file timing & Tx acceptance do not remain identical across executions.

##### Primary-Path Inter-Frame Cadence

For `TARGET_FPS = 30`, the nominal frame period is `33.333 ms`. Over the steady `10 -> 285` interval, `Camera.inter_departure_ms` measures source send-start spacing, while each receiving native boundary exposes `inter_arrival_ms` between the first valid packets of successive observed frames.

| Timing Frontier | `B0-R1` Median | `B0-R2` Median | `B0-R3` Median | Campaign `B0` | Difference from `33.333 ms` |
|---|---:|---:|---:|---:|---:|
| `Camera.inter_departure_ms` | `33.439 ms` | `33.512 ms` | `33.401 ms` | `33.439 ms` | `+0.32 %` |
| `SFF1.inter_arrival_ms` | `33.483 ms` | `33.606 ms` | `33.446 ms` | `33.483 ms` | `+0.45 %` |
| `SFF2 Route 0.inter_arrival_ms` | `33.494 ms` | `33.610 ms` | `33.445 ms` | `33.494 ms` | `+0.48 %` |
| `Encoder.inter_arrival_ms` | `33.478 ms` | `33.585 ms` | `33.454 ms` | `33.478 ms` | `+0.43 %` |
| `SFF2 Route 1.inter_arrival_ms` | `33.233 ms` | `33.156 ms` | `33.389 ms` | `33.233 ms` | `-0.30 %` |
| `Decoder.inter_arrival_ms` | `33.242 ms` | `33.220 ms` | `33.391 ms` | `33.242 ms` | `-0.27 %` |
| `SFF2 Route 2.inter_arrival_ms` | `33.338 ms` | `33.077 ms` | `34.041 ms` | `33.338 ms` | `+0.01 %` |
| `SFF3.inter_arrival_ms` | `33.341 ms` | `33.286 ms` | `34.817 ms` | `33.341 ms` | `+0.02 %` |
| `User.inter_arrival_ms` | `33.082 ms` | `32.971 ms` | `33.673 ms` | `33.082 ms` | `-0.75 %` |

Campaign-central medians remain within approximately `0.8 %` of the nominal period. Individual downstream medians are not expected to invert directly into the global source rate because persistent codec output, queue servicing, & packet bursts redistribute adjacent arrival gaps. The long-window interval means remain close to one nominal period, so the chain preserves source rate without cumulative drift while retaining observable local jitter. The strong lag-3 modulation present in `B0-R3` is discussed in Section 21.6; its absence from `R1`, `R2`, & `BQ` prevents it from being treated as a structural three-frame rule.

| Tx Pressure | `B0-R1` | `B0-R2` | `B0-R3` |
|---|---:|---:|---:|
| Frames with zero accepts | `295` | `296` | `236` |
| Zero-accept events | `497,075` | `523,783` | `224,062` |
| Re-presented packets | `15,736,054` | `16,565,689` | `7,058,348` |
| Partial accepts | `0` | `0` | `0` |
| `mbuf` starvation | `0` | `0` | `0` |

The zero-accept counter remains a sensitivity indicator rather than a correctness predicate. Its run-to-run range is substantial, yet all three source traces preserve 300 / 300 Tx completion & the same absolute source schedule.

##### Active-Processing Repeatability & Downstream Phase Variability

The steady-state median `active_process_ms` values are:

| Node / Route | `B0-R1` | `B0-R2` | `B0-R3` | Campaign `B0` | Run-Level Coefficient of Variation |
|---|---:|---:|---:|---:|---:|
| `Camera` | `18.068` | `18.193` | `15.183` | `18.068` | `9.93 %` |
| `SFF1` | `12.366` | `12.470` | `10.627` | `12.366` | `8.76 %` |
| `SFF2 Route 0` | `6.196` | `6.179` | `3.764` | `6.179` | `26.01 %` |
| `Encoder` | `17.684` | `17.588` | `40.290` | `17.684` | `51.93 %` |
| `SFF2 Route 1` | `0.009` | `0.009` | `0.011` | `0.009` | `11.95 %` |
| `Decoder` | `37.636` | `37.655` | `38.093` | `37.655` | `0.68 %` |
| `SFF2 Route 2` | `0.618` | `0.620` | `0.648` | `0.620` | `2.69 %` |
| `SFF3` | `1.057` | `1.094` | `1.075` | `1.075` | `1.74 %` |
| `User` | `2.332` | `3.326` | `2.914` | `2.914` | `17.47 %` |

The triplicate set separates **semantic repeatability** from local scheduling repeatability. `Decoder`, Route 2, & the final aware boundary remain tight, while `Encoder` exhibits a markedly different writer / codec phase in `B0-R3`. `SFF1.geometry_aggregation_ms` medians are `6.921 / 6.962 / 5.966 ms`, with `max_r_ms = 2.732 / 2.749 / 2.596 ms`; complete offloaded frames retain `Encoder.geometry_aggregation_ms = 0` & `Encoder.max_r_ms = 0` throughout the set.

`B0-R3` is the clearest example of asynchronous redistribution. Its `codec_write_ms` median rises to `30.763 ms`, versus `8.017 / 8.018 ms` in `R1 / R2`, & `encode_h265_ms` reaches `49.130 ms` rather than approximately `32.3 ms`. Nevertheless, `frame_backlog = 0`, `codec_backlog <= 2`, write `EAGAIN = 0`, the steady workload ratio remains below `0.196`, & the controller stays in `IDLE`. This trace also contains no matched "Pose" command. The variation is consequently an explicit host / persistent-codec phase observation, not a change in application semantics.

##### Interaction-Rich Baseline Trace

`B0-R1` contains `52` matched "Pose" commands, compared with `5` in `B0-R2` & none in `B0-R3`. The corresponding positive browser frame-acknowledgment counts are `95 / 110 / 117`; those counts describe rendered snapshots, not browser frame rate or command count. Detailed command statistics are reported once in Section 21.9. One of the 52 `B0-R1` native command matches does not receive a positive `cmd_photon_ms` sample before experiment closure, so browser acknowledgment completeness must not be inferred from native pose convergence alone.

##### Terminal Latency & Run-to-Run Validity

The native `User.reference_e2e_ms` frontier remains complete in every repetition while exposing a bounded host-level range:

| Execution | All-Frame Median | Steady Median | Steady P95 | Steady Max |
|---|---:|---:|---:|---:|
| `B0-R1` | `211.352 ms` | `211.222 ms` | `224.632 ms` | `230.793 ms` |
| `B0-R2` | `214.176 ms` | `214.365 ms` | `225.555 ms` | `233.969 ms` |
| `B0-R3` | `218.310 ms` | `217.313 ms` | `247.062 ms` | `254.926 ms` |

The all-frame medians span `6.958 ms` with a run-level coefficient of variation of approximately `1.63 %`; the steady medians span `6.092 ms` with a coefficient of variation near `1.42 %`. Those values are not presented as zero-variance behaviour. Instead, the triplicate evidence demonstrates that source cadence, frame completion, protocol integrity, final point population, & the overall latency order remain stable while codec scheduling can shift the exact terminal value by several milliseconds.

Against the supplied reference trace, the three all-frame medians correspond to observed reductions of `38.16 %`, `37.34 %`, & `36.13 %` relative to the `341.785 ms` Camera-to-Client median. The campaign headline is:

```text
Reference median Camera -> Client reception = 341.785 ms
Repeated B0 Camera -> User frame-ready      = 214.176 ms
Observed median reduction                   = 37.34 %
B0 all-frame run range                      = 211.352 -> 218.310 ms
```

The repeated result establishes the system-level gain. Within the proposed architecture, "DPDK" supplies a uniform packet-processing substrate & "SFC" supplies explicit steering / control, while the dedicated geometry ablation targets the principal additional computational mechanism: moving reusable progressive & frame-final geometry into `SFF1` so that `Encoder` enters projection with that frontier already resolved.

#### Completed Runtime Sweep — Cross-Run View

The table below uses the steady `10 -> 285` window & reports terminal latency only over complete `User` frames. The best complete runtime baseline is `B0-R1`; repeated `B0` medians remain documented to quantify run-to-run variability. Conditions that change source cadence, admission, or terminal completeness are interpreted through that operating-point change rather than forced into a single scalar ranking.

| Run | Source Cadence | Source Frames Transmitted | Complete `User` Frames | `current_skip` | Steady `User.reference_e2e_ms` Median | Primary Interpretation |
|---|---:|---:|---:|---|---:|---|
| `B0-R1` | `30.003 fps` | `300` | `300` | `1` | `211.222 ms` | best complete runtime comparator |
| `B0-R2` | `30.000 fps` | `300` | `300` | `1` | `214.365 ms` | repeated runtime variability |
| `B0-R3` | `29.996 fps` | `300` | `300` | `1` | `217.313 ms` | repeated runtime variability |
| `O-OFF-R1` | `30.001 fps` | `300` | `300` | `1` | `235.305 ms` | fair local-geometry fallback, high Encoder pressure |
| `O-OFF-R2` | `30.001 fps` | `300` | `300` | `1` | `216.298 ms` | fair local-geometry fallback, schedulable but higher Encoder pressure |
| `C-BEST` | `30.000 fps` | `300` | `300` | `1` | `207.166 ms` | application-resident source bound |
| `C-M-COLD` | `7.882 fps` | `300` | `300` | `1` | `447.064 ms` | cold-page source bottleneck |
| `C-W-WARM` | `29.999 fps` | `300` | `300` | `1` | `215.014 ms` | allocation-heavy warm bound remains schedulable |
| `C-W-COLD` | `7.880 fps` | `300` | `300` | `1` | `444.589 ms` | cold-page bottleneck dominates allocation policy |
| `P84` | `30.002 fps` | `300` | `300` | `1` | `216.998 ms` | near-`MTU` aggregation remains stable |
| `M1000` | `29.999 fps` | `300` | `300` | `1` | `217.358 ms` | finer Route-1 packetisation remains stable |
| `P40` active | `27.172 fps` admitted across the steady span | `270` | `246` | `1 / 2` | `284.807 ms` | packet-rate overload activates control & exposes final-hop pressure |
| `P40-NO-T` | `30.012 fps` | `300` | `258` | `1` | `577.899 ms` | same packet-rate stress without regulation; deeper terminal pressure |

The command column used in earlier drafts is retained only in raw telemetry, because browser interaction traces were manually generated & differ among runs. The updated runtime comparison is now driven by frame completion, source cadence, workload state, & terminal latency rather than by the number of matched interaction events in a given run.

#### Source Residency Sensitivity — `C-BEST`, `C-M-COLD`, `C-W-WARM`, & `C-W-COLD`

The source-residency sweep supplies direct evidence that **warmth governs whether the Camera can sustain the nominal cadence**, whereas `BEST` / `WORST` mainly bound how source work is placed around that condition.

| Source Metric | Campaign `B0` | `C-BEST` | `C-M-COLD` | `C-W-WARM` | `C-W-COLD` |
|---|---:|---:|---:|---:|---:|
| `disk_io_ms` median | `3.309` | `0.000` | `114.810` | `3.636` | `115.212` |
| `active_process_ms` median | `18.068` | `6.898` | `123.713` | `13.813` | `123.854` |
| `tx_duration_ms` median | `11.929` | `6.898` | `6.472` | `7.304` | `6.444` |
| Steady selected-frame cadence | `~30.00 fps` | `30.000 fps` | `7.882 fps` | `29.999 fps` | `7.880 fps` |

`C-BEST` removes timed file acquisition & serialisation by construction, reducing the reported `Camera.active_process_ms` aggregate by approximately `61.8 %` relative to the campaign `B0` value while preserving all 300 frames. This source-local result is real, but `C-BEST` must not be interpreted as a whole-pipeline optimum caused solely by cache policy: its Camera -> `Encoder` frontier is actually higher ( `105.746 ms` versus `96.575 ms` for campaign `B0` ), & its Encoder active / writer medians rise to `38.452 / 28.595 ms`. The Camera -> `Decoder` frontier remains slightly higher at `199.044 ms`, while the terminal `207.166 ms` becomes lower only after that point. The run therefore establishes an optimistic **source-work bound**, not a causal claim that moving all preparation outside the loop yields the observed terminal reduction.

`C-W-WARM` provides the complementary implementation bound. Per-frame allocation / release is present, yet the resident file-backed working set still permits approximately 30 fps. Its lower reported `Camera.active_process_ms` value must not be interpreted as evidence that `WORST` is intrinsically faster: Tx acceptance is lighter than in `B0-R1 / R2`, & the downstream phase is substantially heavier. Encoder active / writer medians rise to `39.510 / 29.663 ms`, Camera -> `Encoder` reaches `109.160 ms`, & Camera -> `Decoder` reaches `203.984 ms`, yet the terminal median remains close to the repeated baseline at `215.014 ms`. This downstream convergence is another example of asynchronous phase redistribution rather than an allocation-speed advantage.

The two cold-page conditions are decisive. `C-M-COLD` & `C-W-COLD` both raise `fread()` to approximately `115 ms`, drive the reported `Camera.active_process_ms` aggregate above `123 ms`, & reduce the selected-frame cadence to about `7.88 fps`. Their near identity shows that once page residency is removed, storage / page-fault acquisition dominates the difference between reusable staging & per-frame allocation. `SFF1` schedule delay grows to approximately `28.05 s` / `28.01 s` by the end of the sequence. All 300 frames eventually complete, but these are **not equivalent real-time 30-fps operating points**.

Downstream behaviour confirms the source-limited regime. `C-M-COLD` / `C-W-COLD` reach Camera -> `Encoder` medians of `173.942 / 170.578 ms` & Camera -> `Decoder` medians of `446.567 / 443.727 ms`; terminal medians are `447.064 / 444.589 ms`. Their Encoder workload ratios fall to approximately `0.130 / 0.131` because the chain is fed only about once every `127 ms`, while `Decoder.decode_h265_ms` expands to approximately `392.178 / 392.913 ms`. That Decoder quantity is a pipelined latency frontier rather than a per-frame decode-throughput cost, so its expansion reflects the changed temporal operating point rather than fourfold reconstruction work.

The source sweep therefore supports a precise hierarchy: **page residency is necessary on this host to sustain the target source cadence; allocation policy is secondary once pages are resident; `BEST` & `WORST` are sensitivity bounds, while `MIDDLE + WARM` remains the most defensible principal runtime condition**.

#### Geometry-Offload Consumption — Fair `O-OFF` Protocol

The fair runtime `O-OFF` reruns pair `SFF1.NETWORK_PROCESSING_DISABLED` with `Encoder.OFFLOAD_MODE_DISABLED`. `SFF1` continues to classify / encapsulate / forward the same packets & service state but skips progressive & final geometric scans; `Encoder` then reconstructs the reference-compatible centroid / extrema / `max_r` / `final_scale` / transformed-frontier / `global_scale` sequence once, before entering the unchanged "CUDA" branch. This is the mechanism-level experiment used to quantify the contribution of **in-network geometry placement** while retaining the common "DPDK" execution substrate, "SFC" steering, source configuration, persistent "codec" state, & workload-controller parameters.

| Encoder / Placement Metric | `B0-R1` | `O-OFF-R1` | `O-OFF-R2` | Reading |
|---|---:|---:|---:|---|
| `SFF1.geometry_aggregation_ms` | `6.921 ms` | `0.000 ms` | `0.000 ms` | in-path geometry removed by the fair ablation |
| `SFF1.max_r_ms` | `2.732 ms` | `0.000 ms` | `0.000 ms` | exact radius removed from `SFF1` |
| `Encoder.geometry_aggregation_ms` | `0.000 ms` | `5.295 ms` | `4.828 ms` | equivalent geometry reconstructed locally |
| `Encoder.max_r_ms` | `0.000 ms` | `2.472 ms` | `2.427 ms` | exact radius paid at `Encoder` |
| `Encoder.active_process_ms` | `17.684 ms` | `54.335 ms` | `26.081 ms` | local fallback consumes more Encoder headroom |
| `workload_ratio` median | `0.165` | `0.419` | `0.391` | both fair attempts operate with less headroom |
| steady `User.reference_e2e_ms` median | `211.222 ms` | `235.305 ms` | `216.298 ms` | in-network placement remains the best runtime result |

Both fair attempts complete `300 / 300` source frames at nominal source cadence & keep `current_skip = 1`. The first attempt shows a larger writer / codec phase displacement, while the second is closer to the baseline latency; both nevertheless move the geometry frontier away from `SFF1` & increase Encoder workload pressure. This is the final placement reading used by the updated campaign. The earlier archive in which `SFF1` still computed geometry while `Encoder` repeated it remains a historical duplicated-work diagnostic, not the current attribution basis.

#### Capture-Enabled Offload Control — `BQ` vs. `OQ-OFF`

`OQ-OFF` completes all 300 source frames at approximately 30 fps & `current_skip = 1` under the same fair geometry placement used by the runtime `O-OFF` pair: `SFF1` performs no progressive / final geometry, while `Encoder` reconstructs the equivalent frontier exactly once. The direct steady Encoder medians are `6.039 ms` for geometry aggregation / transformed-frontier work & `3.348 ms` for `max_r`; `Encoder.active_process_ms` rises from `24.140 ms` in `BQ` to `37.493 ms`, while the workload-ratio median rises from `0.167` to `0.465` ( maxima `0.208` versus `0.499` ). No controller transition occurs.

The same run also has a somewhat heavier writer / codec phase: `codec_write_ms = 18.043 ms` & `encode_h265_ms = 41.054 ms`, versus `15.508 / 39.507 ms` in `BQ`. Camera -> `Encoder`, Camera -> `Decoder`, & Camera -> `User` steady frontiers are `102.098 / 196.670 / 196.981 ms` in `OQ-OFF`, versus `101.577 / 196.729 / 196.820 ms` in `BQ`. The terminal medians therefore remain close even though the local placement cost at `Encoder` is explicit; the all-frame `OQ-OFF` P95 grows to `226.341 ms`, so this capture-enabled pair remains a fidelity / placement control rather than the browser-enabled runtime ranking.

All 300 final `valid_points` values match `BQ` frame by frame. Sequence-level `MSE-Y`, `PSNR-Y`, `SSIM-Y`, & `Gauge` values remain materially unchanged as quantified in Sections 21.10 & 21.11. The final capture-enabled control therefore supports the same conclusion as the runtime reruns: **removing the in-network geometry frontier consumes more `Encoder` headroom without producing a material reconstructed-content shift at the measured quality resolution**.

#### Near-`MTU` Point Aggregation — `P84`

`P84` preserves `300 / 300` completion, `100 %` integrity, `current_skip = 1`, zero protocol errors, & the same `43,620,972` final valid points as the repeated baseline. Increasing point capacity from `80` to `84` lowers the full-run source datagram count from `2,976,979` to `2,835,230` ( `-4.76 %` ) & the reconstructed-point count from `545,406` to `519,441` ( `-4.76 %` ).

Point-path service work remains stable: `SFF1.active_process_ms` moves from `12.366` to `12.310 ms`, Route 0 from `6.179` to `6.062 ms`, Encoder active processing from `17.684` to `17.745 ms`, & Decoder active processing from `37.655` to `37.755 ms`. More importantly, the progressive Camera -> `Encoder` / Camera -> `Decoder` frontiers are `96.485 / 196.537 ms`, effectively unchanged from the `96.575 / 196.904 ms` campaign comparators. The terminal median is `216.998 ms`, `2.633 ms` above the campaign `B0` central value but still within the repeated-baseline steady envelope ( `211.222 -> 217.313 ms` ).

The downstream profile therefore does not support a packet-capacity latency penalty. The small terminal increase emerges after a nearly unchanged Decoder frontier & coincides with somewhat higher `SFF3` / `User` active medians in this single execution. `84` is consequently supported as a safe near-`MTU` operating point that reduces datagram frequency without fragmentation, control activation, frame loss, or a material processing penalty.

#### Compressed-Media Grouping — `M1000`

`M1000` changes only the application-level compressed-media chunk size. The `188 B` "MPEG-TS" unit & Encoder parser remain unchanged. All 300 application frames complete at `current_skip = 1`, with the same final point population & zero codec / protocol errors.

Reducing `MEDIA_PAYLOAD_SIZE` from `1316` to `1000` increases the Route-1 datagram population from approximately `7,995` in the repeated `B0` set to `10,498` ( approximately `+31.3 %` ). Route-1 active processing remains `0.010 ms` versus `0.009 ms` for campaign `B0`, while its median protocol-inclusive bitrate rises from approximately `7.364` to `7.551 Mbit / s` because more network headers are required.

The progressive Camera -> `Encoder` / Camera -> `Decoder` medians remain `96.828 / 196.960 ms`, only `+0.254 / +0.056 ms` relative to the repeated comparator. The terminal median reaches `217.358 ms`, approximately `+2.993 ms` relative to campaign `B0` & only `0.045 ms` above the upper repeated-baseline median. Since the shift appears after an essentially unchanged Decoder frontier while codec, final points, & completion remain stable, it is more defensible to attribute the controlled effect to higher header / packet frequency than to claim an intrinsic decoding-latency regression. The run validates segmentation / reassembly across arbitrary `188 B` transport-packet boundaries.

#### `P40` — Packet-Rate Stress, "Temporal" Activation, & Downstream Capacity Boundary

`P40` remains qualitatively different from the other sensitivity points. Halving point capacity approximately doubles the point-datagram rate per admitted frame. After temporal admission suppresses 30 source frames, the active run still carries `5,356,462` source point datagrams & `981,801` reconstructed-point datagrams, compared with `2,976,979` & `545,406` in a complete `B0` execution.

The increased packet frequency raises work across the point-processing frontiers. Relative to the repeated `B0` medians, `SFF1.active_process_ms` rises from `12.366` to `13.030 ms`, Route 0 from `6.179` to `6.689 ms`, Decoder active processing from `37.655` to `41.096 ms`, Route 2 from `0.620` to `0.914 ms`, & `SFF3` from `1.075` to `1.674 ms`. Encoder active processing rises to `44.385 ms`; its workload ratio reaches `1.000` & codec backlog reaches `4`.

Exactly three overload / recovery cycles are recorded:

```text
Encoder SKIP+1 requests  : frame 76, 174, 270
Camera applies skip = 2  : frame 78, 177, 273
Encoder SKIP-1 requests  : frame 97, 197, 293
Camera returns to skip 1 : frame 98, 198, 294
```

At the three `SKIP+1` rows, Encoder workload ratios are `0.920`, `0.966`, & `0.901`, with `frame_backlog = 1` & `codec_backlog = 4`. Recovery rows report `0.092`, `0.094`, & `0.094`. The source consequently transmits `270` of the 300 frame IDs, & the `365` codec rows corroborate `90 + 270 + 5`.

The active controller events are:

| `P40` Controller Event | Frame | Active `skip` | `workload_ewma_ms` | `workload_ratio` | `raw_queue_ms` | `frame_backlog` | `codec_backlog` |
|---|---:|---:|---:|---:|---:|---:|---:|
| `SKIP+1` | `76` | `1` | `30.651` | `0.920` | `10.083` | `1` | `4` |
| `SKIP-1` | `97` | `2` | `6.136` | `0.092` | `0.002` | `0` | `1` |
| `SKIP+1` | `174` | `1` | `32.209` | `0.966` | `0.403` | `1` | `4` |
| `SKIP-1` | `197` | `2` | `6.249` | `0.094` | `0.009` | `0` | `1` |
| `SKIP+1` | `270` | `1` | `30.035` | `0.901` | `12.541` | `1` | `4` |
| `SKIP-1` | `293` | `2` | `6.291` | `0.094` | `0.002` | `0` | `1` |

Across the complete admitted Encoder trace, `workload_ratio` reaches `1.000` ( median `0.194` ), `raw_queue_ms` reaches `16.718 ms` while retaining a `0.002 ms` median, `frame_backlog` remains bounded to `1`, & `codec_backlog` reaches `4` with a median of `3`. The chronology is directly consistent with the controller definition in Section 9.6: workload / queue pressure accumulates at `skip = 1`, the returned `Camera` state reflects `skip = 2`, queue pressure collapses, & the configured recovery streak returns the source to `skip = 1`.

This remains a **successful validation of the reverse "Temporal" control loop under real overload**. The admitted frame set is preserved despite observable upstream Tx pressure. `Camera` records `1,550,251` zero-accept attempts across its `270` selected frames, while `SFF2 Route 0` records `18,521` zero accepts across three frames; bounded resubmission absorbs both conditions without `mbuf` starvation, Route 0 partial accepts remain `0`, & every admitted frame stays complete through `SFF2 Route 2`. `SFF1`, `Encoder`, `SFF2 Route 1`, `Decoder`, & `SFF2 Route 2` record no Tx zero-accept / partial-accept events in this run.

The first application-visible completeness break appears at the final `SFF3` -> `User` boundary. `SFF3` receives all `270` admitted complete frames but completes `246` terminal transmissions, with `240,942` zero-accept events, `558` partial accepts, & `6,506,264` local re-presented packet attempts. Across the 24 incomplete terminal frames, `1,934` point datagrams / `77,360` reconstructed points are not forwarded beyond the `SFF3` Tx frontier. The discrepancy is therefore localised to final-hop backpressure rather than hidden upstream loss; the available telemetry identifies the `SFF3` -> `User` boundary, without attributing the entire effect to `User` alone.

The no-"Temporal" diagnostic makes the regulation effect concrete. Holding the same `P40` packetisation at `skip = 1` admits all `300` source frames but produces only `258` complete `User` frames. `SFF3` pressure rises to `355,966` zero accepts, `560` partial accepts, & `9,223,565` re-presented packet attempts, versus `240,942 / 558 / 6,506,264` with regulation active. "Temporal" therefore reduces source injection & final-hop pressure, but it does not directly control the `SFF3` -> `User` Tx frontier; that boundary remains the remaining fine-tuning target.

The `284.807 ms` steady median `User.reference_e2e_ms` over complete active-`P40` terminal frames is reported for completeness only. Since the run changes admitted content, alternates between `skip = 1 / 2`, & terminates 24 admitted frames incompletely, it is not used as a nominal latency ranking against the lossless full-rate conditions. The no-"Temporal" diagnostic is even further outside the nominal comparator, with `577.899 ms` steady terminal latency over its complete frames. Both are operating-point / capacity results.

#### Cross-Campaign Interpretation

The completed campaign supports distinct conclusions without forcing every condition into one scalar ranking:

```text
B0                         : complete nominal-rate runtime reference with measurable host / codec-phase variability
BQ                         : complete nominal-quality fidelity reference
BEST                       : optimistic source bound when preparation is removed from the timed loop
MIDDLE / WORST ( no WARM ) : page residency loss collapses source cadence to about 7.88 fps
WORST + WARM               : pessimistic allocation semantics remain schedulable with resident pages
O-OFF-R1 / R2              : fair placement ablation disables SFF1 geometry & executes the frontier once at Encoder
OQ-OFF                     : fair capture-enabled placement control preserves 300-frame fidelity at skip = 1
P84                        : near-"MTU" aggregation reduces point-datagram count without destabilisation
M1000                      : finer media grouping increases header frequency without reassembly failure
P40                        : packet-rate stress activates "Temporal" & exposes the SFF3/User capacity boundary
P40-NO-T                   : same packet-rate stress without source regulation deepens final-hop pressure
```

The methodological separation is five-way. `C-BEST`, `C-W-WARM`, `P84`, & `M1000` retain nominal source cadence, `skip = 1`, & complete terminal delivery. `C-M-COLD` / `C-W-COLD` preserve all content but operate at a source-limited cadence near `7.88 fps`. The fair runtime `O-OFF` reruns preserve 300-frame completion while raising Encoder workload pressure. Active `P40` changes both admission & terminal completeness, while `P40-NO-T` isolates the same packet-rate pressure without source regulation. `BQ` / fair `OQ-OFF` are capture-enabled fidelity controls & must not be substituted for browser-enabled runtime ranking.

The repeated baseline remains the correct denominator for stable full-rate runtime comparisons. Where a variant changes source cadence, temporal state, completeness, or measurement mode, the transition itself becomes the primary result & terminal latency is interpreted descriptively.

#### Fidelity Scope

`BQ` supplies objective fidelity for the common in-network configuration, while `OQ-OFF` applies the fair no-in-network-geometry placement with `QUALITY_CAPTURE = 1`. Both contain all 300 frame IDs at `current_skip = 1`; their final `valid_points` vectors are identical, while aggregate "luma" & `Gauge` differences remain small at the reported resolution. The frame-aligned deltas in Sections 21.10 & 21.11 additionally expose the residual per-frame spread rather than collapsing it into sequence means. This is direct evidence that **the measured in-network placement changes workload headroom without producing a systematic material fidelity shift**.

Repeating "PSNR", "SSIM", & `Gauge` for every packetisation level is unnecessary when the corresponding runtime execution preserves frame completion, final point population, codec-drop counters, & `current_skip = 1`. `P84` & `M1000` satisfy those invariants & remain packetisation tests rather than rate-distortion experiments.

`P40` does **not** satisfy the same terminal invariant: source admission changes & 24 admitted frames are incomplete at `User` in the active run. A capture-enabled `P40` run would answer a different question — quality under overload / selective delivery — & is not required to validate the nominal-quality operating point.

---

## ⚠️ 22. Experimental Interpretation & Known Boundaries

### 22.1 Logical Bytes vs. Wire Bytes

Logical payload, full application datagram, & reference-size bitrate answer different questions. "Ethernet" framing, service metadata, "UDP" / "IP" headers, reconstructed-point payloads, & compressed-media bytes must not be interchanged when reporting throughput.

### 22.2 `CACHE_MODE` & `WARM_MODE` Are Part of the Experimental Condition

The final source condition is:

```text
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

Changing either value defines a different benchmark. The runtime sweep shows why both must be reported: disabling warm mode reduces `MIDDLE` & `WORST` source cadence to approximately `7.88 frames / s`, whereas their warmed counterparts remain near the 30-fps target. `BEST` & `WORST` are retained as source sensitivity bounds rather than substitutes for the principal `MIDDLE + WARM` condition.

### 22.3 Runtime & Fidelity Measurements Must Remain Separate

`QUALITY_CAPTURE = 0` measures the interactive system with the Web bridge / browser, whereas `QUALITY_CAPTURE = 1` replaces that presentation path with in-memory fidelity capture & post-stream analysis. Browser "CTP" belongs only to the former condition; "PSNR", "SSIM", & `Gauge` geometry belong only to the latter. `BQ` / `OQ-OFF` are consequently valid fidelity controls but are not performance substitutes for the repeated browser-enabled `B0` runtime comparator.

### 22.4 Source Scheduling, Descriptor Depth, & Backpressure Are Joint Variables

The final 4096-descriptor configuration is a fixed experimental condition. `Camera` zero-accept volume demonstrates that a complete 300-frame run can still contain substantial local producer / consumer pressure. Changing queue depth, retry bounds, affinity, or source-cache behaviour alters the operating point.

### 22.5 Core Affinity Is Part of the Experiment

All eight logical cores are used by the final application / "codec" layout across the stream phase. The former "OVS" "PMD" core is not an independent hidden resource; it has been reassigned to application work. "SMT" sibling sharing & Core `0` housekeeping must be retained in comparative reports.

### 22.6 Complete Route 2 Is Validated

Unlike earlier snapshots, `Decoder -> SFF2 -> SFF3 -> User` now possesses a stable `dec_hdr` contract, exact point accounting, route-specific telemetry, service re-encapsulation, & final decapsulation. Every `B0`, `BQ`, `OQ-OFF`, & fair `O-OFF` complete reference / placement / fidelity execution preserves 300 / 300 terminal completion; active `P40` deliberately exceeds the final-hop capacity envelope & is analysed separately as the stress-boundary exception.

### 22.7 End-to-End & "CTP" Are Available but Distinct

Native `User` `e2e_latency_ms` measures the propagated `Camera` timestamp to `User` node exit. Browser `cmd_photon_ms` measures a control request to the first matching browser render-completion proxy immediately after `renderer.render()` returns; it is not a physical display photon measurement. These are different causal frontiers & must not be summed or substituted for one another.

### 22.8 "NSH" Interoperability Is Not Claimed

The project deliberately uses an "NSH"-inspired closed contract. The experimental metadata class / next-protocol choices do not establish generic interoperability with arbitrary "RFC 8300" implementations.

### 22.9 "Temporal" State & Workload Evidence

`TEMPORAL_ADAPTATION_ENABLED` is active throughout all three `B0` repetitions, `BQ`, & `OQ-OFF`. The three runtime baselines & both quality controls remain at `current_skip = 1` for all 300 frames. The archived duplicated-work `O-OFF` records one `skip 1 -> 2 -> 1` cycle & suppresses 11 source frames while preserving every admitted frame; `P40` records three cycles, admits 270 source frames, & additionally exposes incomplete delivery at the `SFF3` -> `User` frontier.

The controller is deterministic over its measured inputs. The repeated nominal `B0` traces remain below the overload region, with steady maximum `workload_ratio` values no higher than approximately `0.196`. The fair runtime `O-OFF` reruns raise the workload ratio but remain complete at 300 / 300, while the fair capture-enabled `OQ-OFF` control reaches `0.499` without crossing the transition frontier. The packet-rate `P40` run reaches `1.000` & exercises the full increase / recovery loop. The final runtime placement conclusion is therefore taken from the fair `SFF1.NETWORK_PROCESSING_DISABLED + Encoder.OFFLOAD_MODE_DISABLED` reruns under the same browser-enabled runtime condition & unchanged workload-controller parameters.

### 22.10 "Codec" Diagnostics Are Not Application Frame Tables

Persistent pre-roll / post-roll deliberately injects private chronology into `ffmpeg.txt`. Native application ".csv" attribution must remain authoritative.

### 22.11 Objective Geometry Depends upon the Stated Gauge Procedure

The millimetre metrics include pose reversal, statistical filtering, robust "ICP" alignment, & nearest-neighbour comparison using `VOXEL_MM = 1.820`. They are therefore **post-registration shape-fidelity metrics** of the documented evaluation pipeline, not unqualified raw point-index differences & not independent measurements of residual global pose / rigid-registration error.

### 22.12 Dataset Artefacts Are External to Git

The repository intentionally excludes the multi-gigabyte "PLY" / "BIN" sequence. Reproducibility requires independent dataset acquisition plus the exact Converter settings.

### 22.13 Reference Comparison & Architectural Attribution

The direct values in Section 21.15 combine the current archived runs, the supplied probe-disabled reference telemetry, & published values from the reference thesis. The best repeated `B0` trace measures a `211.352 ms` Camera-to-User frontier against the `341.785 ms` reference value, while preserving the same 300-frame / 30-fps service objective. Within the architecture, "DPDK" provides one uniform user-space packet-processing substrate & "SFC" supplies explicit service steering / reverse control. The principal additional computational mechanism is the movement of reusable geometry from the frame-completing application boundary into `SFF1`'s in-path execution.

The two completed fair `O-OFF` reruns are constructed around that attribution: `SFF1` retains the same forwarding / encapsulation role but performs no progressive or final geometric work, while `Encoder` reconstructs that frontier locally exactly once. Their measured deltas are therefore the direct experiment used to quantify how much `Encoder` headroom is recovered by in-network processing. Platform-level total-"CPU" percentages remain a separate resource-accounting question, but they do not alter the measured placement result itself.

### 22.14 "UDP" Applicability Is Deliberately Restricted

The current "UDP" rationale is valid for the closed "DPDK" / "SFC" experiment because message identity, application pacing, returned-state confirmation, queue pressure, & integrity are explicitly controlled. "RFC 8085" should be treated as a boundary condition rather than ignored: a deployment over the unrestricted Internet would require an appropriate congestion-control / fairness strategy, checksum policy, path-"MTU" consideration, security treatment, & potentially a different transport. The project therefore claims **semantic suitability within its controlled domain**, not universal transport superiority over "TCP".

### 22.15 Frame-Global Projection & "ISC" Boundary

`ISC` ( `Independent Slice Coding` ) was considered explicitly as a possible next encoding refinement. Dividing the already formed Super-Frame into independent "HEVC" slices could improve within-picture isolation & may expose additional codec parallelism, but it does not remove the principal latency boundary of the current application. `Encoder` must first resolve frame-complete geometry, execute the joint six-view G-buffer / visibility pipeline, pack `Geometry`, `Texture`, & `Occupancy` into one complete `I420` picture, & only then submit raw video to `hevc_nvenc`. A slice configuration applied after that point cannot make the earlier point-cloud projection operate packet by packet.

The present packaged "FFmpeg" build also rejects the tested `-constrained-encoding` option as unrecognised. Merely replacing that binary or adding a generic slice-count option would still leave the rawvideo input, "MPEG-TS" attribution, Decoder full-`I420` read, & reconstruction frontiers frame based. Genuine sub-frame latency would instead require a coordinated redesign involving an "NVENC" interface capable of slice-level bitstream readback, incremental muxing / frame association, & a Decoder / reconstruction path able to consume useful partial output. That is a qualitatively different architecture, not another low-risk `execlp()` flag.

Accordingly, no "ISC" change is included in the validated baseline. The current implementation already addresses the latency mechanisms **evaluated within the present scope** that can be modified without changing frame semantics — prompt Encoder muxer flushing, zero Encoder delay, progressive `MEDIA_PAYLOAD_SIZE` emission, persistent low-delay Decoder state, & bounded asynchronous application queues. Additional slice work remains a separate research extension rather than an omitted correction to the involved 30-fps chain.

---

## 🛠️ 23. Major Engineering Challenges & Solutions

### Limited Logical Cores

**Challenge:** Seven native nodes, two persistent "codec" processes, a browser bridge, & host housekeeping must coexist on eight logical "CPU" threads.

**Current approach:** Remove the previous "OVS"-"DPDK" intermediary, pin every native role explicitly, separate `Decoder` "DPDK" from "codec" I/O, share only comparatively light / asynchronous roles, lower "Python" bridge priority, & run `Gauge` after "EOS" rather than concurrently.

### "DPDK" Backpressure at the Source

**Challenge:** The `Camera` can encounter repeated zero-accept Tx returns even when the final stream remains complete.

**Current approach:** Bounded local re-presentation, explicit zero / partial counters, 4096-descriptor queues, & strict distinction between local ring acceptance & "UDP"-level semantics.

### Sustaining the Nominal 30 Frames / s Operating Point

**Challenge:** Source disk access, serialisation, packet submission, in-path geometry, projection, encode, decode, reconstruction, & terminal delivery must coexist without progressive frame loss.

**Current approach:** Middle-cache / warm source condition, persistent "GPU" / "codec" state, direct adjacency, cooperative polling, bounded queues, & complete per-node telemetry. All three `B0` repetitions together with `BQ` & `OQ-OFF` sustain approximately 30.0 source frames / s with 300 / 300 final completion; operating-point sensitivity variants are reported separately when admission or completeness changes.

### Performing Useful Work In-Network

**Challenge:** Identify operations that are mathematically compatible with packet-progressive execution.

**Current approach:** Move associative sums / extrema into `SFF1`, retain the centroid-dependent exact radius boundary at frame completion, & expose validated geometry to `Encoder` through service metadata.

### Protecting Rx While Computing

**Challenge:** Long "CPU" / "GPU" / pipe waits can delay packet servicing.

**Current approach:** Incremental packet placement, cooperative "DPDK" polling, separate "codec" writer roles, persistent buffers, & bounded frame / "codec" queues.

### Maintaining Service State Around Unaware Applications

**Challenge:** `Encoder` & `Decoder` should remain ordinary application parsers.

**Current approach:** `SFF2` captures, advances, removes, & reconstructs service state around both unaware nodes; `SFF3` consumes the final aware primary state.

### Reliable Control Semantics Without a Reliable Byte Stream

**Challenge:** "Pose" & "Temporal" directives are operationally important, yet the native Data Plane deliberately avoids persistent "TCP" state. A lost, duplicated, or delayed control datagram must not leave the application permanently inconsistent.

**Current approach:** Commands encode absolute desired state rather than relative event histories. "Pose" retains one timestamp across re-presentations & `Decoder` suppresses duplicate / older timestamps; "Temporal" retains an absolute requested skip & is re-issued while the returning `Camera` stream has not reflected it. Both paths therefore verify application effect through returned state. This provides the required convergence semantics while preserving one-message / one-datagram classification at the `SFF` frontiers.

### Preserving "MTU" Across Different Traffic Types

**Challenge:** Raw points, geometric metadata, compressed "MPEG-TS" groups, reconstructed points, "Temporal" control, & "Pose" control possess different envelopes.

**Current approach:** Derive each payload limit from its complete datagram structure; the largest final primary aware datagram remains below 1500 B.

### Correctly Attributing Asynchronous "Codec" Output

**Challenge:** Pipe reads & "codec" output do not preserve source frame boundaries.

**Current approach:** Persistent pre-roll, frame-order attribution, "MPEG-TS" / "PES" parsing, explicit decoder frame ordering, private warm-up IDs, & native application ".csv" files separated from `vstats` chronology.

### Avoiding Browser Backlog

**Challenge:** Browser rendering cannot consume every native 30-fps point-cloud snapshot on the constrained test host.

**Current approach:** Latest-frame-only shared publication, peer queue capacity one, payload copy deferred until consumer readiness, one frame in flight, & render-triggered acknowledgment. Native `User` reception therefore remains 300 / 300 even when browser acknowledgments occur at a lower rate.

### Measuring Quality Without Polluting the Runtime Experiment

**Challenge:** Disk writes, "ICP", "KD-Tree" searches, & quality filters can materially disturb a real-time benchmark.

**Current approach:** Bounded in-memory capture during QUALITY, persistent serialization after "EOS", serial `Gauge` execution only after `User` readiness, & a separate NON-QUALITY run for interaction / "CTP". The retained `User` design intentionally stages reconstruction in `web_points` & copies only complete frames into `quality_buffer`; a direct-write capture variant was experimentally rejected after producing unacceptable host-level instability, so it is explicitly excluded from the validated measurement configuration.

---

## ✅ 24. Reproducibility Checklist

Every archived benchmark should preserve at least:

```text
source revision
README revision
host kernel
"CPU" model & "SMT" topology
"GPU" model
NVIDIA driver
"GCC" / "G++" version
"NVCC" version
"CUDA" version
"DPDK" version
"Docker" version
"FFmpeg" version
"Python" / "NumPy" / "SciPy" / "websockets" versions

"HugePages" count & size
"Docker" cpusets
`isolcpus` state verified from `/proc/cmdline`
native "DPDK" lcores
Encoder FFMPEG_CPU
Decoder FFMPEG_CPU
CPU_ARCH / CUDA_ARCH & effective compiler flags
User bridge affinity / nice level
QUALITY_CAPTURE

Camera CACHE_MODE
Camera WARM_MODE
TARGET_FPS
K_FRAMES
POINTS_PER_PACKET
NETWORK_MTU
MEDIA_PAYLOAD_SIZE
BURST_SIZE
MAX_ZERO_ACCEPTS
Rx / Tx descriptor depth
virtio queue_size

MAIN_SPI / SI transitions
TEMPORAL_SPI / SI
POSE_SPI / SI
DEFAULT_TTL / metadata class / next-protocol values
exact "Main" / "Temporal" / "Pose" payload sizes
control retry interval & returned-state confirmation rules
"UDP" checksum convention & controlled-environment applicability
SFF2 proxy rules
Route-0 / Route-1 / Route-2 byte semantics

Encoder OFFLOAD_MODE
Encoder TEMPORAL_ADAPTATION
MAX_SKIP
EWMA_ALPHA
OVERLOAD_RATIO
RECOVERY_RATIO
OVERLOAD_STREAK
RECOVERY_STREAK
RETRY_FRAMES
H2D_CHUNK_POINTS
YUV_BUFFER_COUNT
QUALITY_STREAM_SIZE
target "H.265" bitrate / buffer
"GOP" / "IDR" configuration
Encoder delay / flush-packets configuration
Decoder low-delay configuration
DEBUG_VISUALS / DEBUG_FRAME_ID when diagnostic snapshots are enabled

Decoder QUEUE_SIZE
WRITE_BATCH_SIZE
I420_BUFFER_COUNT
hardware decode configuration
reconstruction / occupancy dimensions
pose-control policy

User QUALITY_BUFFER_SIZE
"WebSocket" compression state
peer queue size / one-frame-in-flight policy
`Gauge` `VOXEL_MM` / "ICP" / outlier parameters

all native telemetry ".csv" files
`Encoder` / `Decoder` `ffmpeg.txt` + `stderr.log` files
comparison-baseline revision / log set when reporting cross-architecture percentages
```

The final representative configuration is:

```text
CACHE_MODE          = CACHE_MODE_MIDDLE
WARM_MODE           = WARM_MODE_ENABLED
OFFLOAD_MODE        = OFFLOAD_MODE_ENABLED
TEMPORAL_ADAPTATION = TEMPORAL_ADAPTATION_ENABLED
TARGET_FPS          = 30
K_FRAMES            = 300
POINTS_PER_PACKET   = 80
NETWORK_MTU         = 1500
MEDIA_PAYLOAD_SIZE  = 1316
BURST_SIZE          = 32
MAX_ZERO_ACCEPTS    = 2048
Rx / Tx descriptors = 4096
User queue_size     = 4096
"HugePages"         = 1024 x 2 MiB
isolation helper    = isolcpus=1-7; archive actual `/proc/cmdline` state
```

Core placement:

```text
0 User "DPDK" / post-"EOS" Gauge in QUALITY
1 Camera
2 Decoder "FFmpeg" + writer / lower-priority Web bridge in NON-QUALITY
3 SFF1
4 Decoder "DPDK"
5 Encoder "DPDK"
6 SFF2
7 SFF3 + Encoder "FFmpeg" / writer
```

A benchmark attains reproducibility only when the result files & the exact mode / placement generating them are archived concurrently. Boot-time `isolcpus` state is not encoded inside the native telemetry files, so `/proc/cmdline` ( or an equivalent experiment manifest ) must accompany a result set whenever isolation participates in the benchmark.

---

## 🚧 25. Future Extensions

The functional `Camera`-to-`User` chain is complete, including the `Main` service path, the reverse `Temporal` & `Pose` control paths, the two completed fidelity controls, native per-frame telemetry, browser-side `Command-to-Photon` tracking, & the final validated "CPU"-affinity arrangement. Future work should therefore concern **broader deployment conditions**, **protocol interoperability**, **multi-stream scalability**, & **additional platform-level characterisation**, rather than completion or re-validation of mechanisms already consolidated within the present implementation.

### "Independent Slice Coding" ( "ISC" ) & Sub-Frame Experimentation

`ISC` remains an investigated but deliberately uncommitted extension. The current Super-Frame contains three semantically distinct `2560 x 1536` `Y`-plane stripes ( `Geometry`, `Texture`, & `Occupancy` ), making independent "HEVC" regions conceptually attractive for error isolation or codec-side parallelism. Nevertheless, the present projection kernel produces these representations from the complete point-cloud frame before `FFmpeg` receives any raw video, & the current Decoder does not reconstruct until a complete `I420` picture is available. Slice partitioning alone therefore offers no demonstrated end-to-end latency or quality advantage under the validated architecture; independent boundaries may additionally reduce compression efficiency by restricting prediction across slice frontiers.

A future proof of concept should consequently remain isolated from the stable baseline:

```text
1. build a reproducible "FFmpeg" / "NVENC" environment that actually exposes constrained independent-slice controls
2. verify exact slice boundaries & resulting bitrate / quality before modifying application framing
3. evaluate native sub-frame bitstream readback only if first-slice latency is an explicit research objective
4. redesign "MPEG-TS" attribution & Decoder consumption only if a measurable partial-frame benefit survives the first three steps
```

This ordering prevents a codec flag from being presented as packet-wise projection when the mathematically frame-global geometry / visibility stages remain unchanged.

### Portability & Deployment

```text
1. repeat the complete experiment on hosts exposing a different number of physical / logical cores
2. validate the final "CPU"-affinity policy across heterogeneous "SMT" / "NUMA" topologies
3. evaluate deployment across physically distributed hosts rather than the current controlled local topology
4. quantify the sensitivity of the validated service chain to heterogeneous compute resources across edge / cloud placement conditions
```

These experiments would establish how strongly the current operating point depends upon the characteristics of the reference host, while preserving the already validated service semantics, packet formats, & node responsibilities.

### Protocol Interoperability

```text
1. evaluate fully standards-compliant "NSH" encapsulation if interoperability with external "SFC" implementations becomes a research requirement
2. validate the closed packet representation on architectures exposing different host endianness / alignment characteristics
3. revisit checksum, congestion-control, & transport assumptions before any unrestricted wide-area or Internet-facing deployment
4. assess compatibility with external service-chain classifiers / orchestrators without altering the internal behaviour of "SFC"-unaware functions
```

The packet representation remains intentionally `"NSH"-inspired` & closed within the experimental topology; interoperability with generic "RFC"-compliant service-chain infrastructures is therefore a distinct extension rather than a missing element of the present implementation.

### Multi-Stream Scalability & Orchestration

```text
1. introduce an explicit stream / session identifier if concurrent multi-user multiplexing is required
2. evaluate whether selected "SFC" functions can be shared across concurrent sessions while preserving per-stream service state
3. measure replicated-chain vs shared-function deployment strategies under controlled multi-stream load
4. quantify orchestration decisions in terms of placement granularity, resource assignment, service migration cost, & request admission capacity
```

The present architecture already exposes a finer placement granularity than the monolithic reference blocks; however, true multi-stream scalability requires explicit stream identity & controlled contention experiments before quantitative claims regarding concurrent-user capacity can be made.

### Extended Resource Characterisation

```text
1. complement the existing per-node latency / residency / efficiency telemetry with platform-level "CPU" utilisation measurements
2. introduce "GPU" utilisation & memory-bandwidth observations at the projection / reconstruction stages
3. measure power / energy consumption where suitable hardware instrumentation is available
4. relate resource consumption to placement decisions across different orchestration scenarios
```

This extension would allow the currently measured latency, throughput, & quality profiles to be complemented by a complete compute-resource cost model, enabling more direct use of the pipeline as an orchestration input within heterogeneous compute-continuum scenarios.

### Broader Application Validation

```text
1. repeat the validated pipeline on additional volumetric datasets exhibiting different point densities, motions, & spatial distributions
2. assess the portability of the fixed packetisation, codec, & reconstruction parameters across such datasets
3. evaluate the integration of an actual "VR" / "AR" terminal while preserving the existing browser / shared-memory diagnostic path as a reference condition
```

These experiments would test the generality of the current implementation beyond the `Loot` sequence without changing the architectural principles validated in the present work.

The immediate research priority is therefore no longer architectural completion or re-validation of the current service chain; it is the extension of an already complete & experimentally characterised "SFC" pipeline toward **portable deployment**, **standards-oriented interoperability**, **multi-stream orchestration**, **broader resource-aware evaluation**, & — only where independently justified — **sub-frame codec experimentation**.

---

## 📚 26. References


1. J. Halpern & C. Pignataro, **"Service Function Chaining" ( "SFC" ) Architecture**, "RFC 7665", "IETF", 2015.
2. P. Quinn, U. Elzur, & C. Pignataro, **"Network Service Header" ( "NSH" )**, "RFC 8300", "IETF", 2018. The present project adopts its "SPI" / "SI" terminology & architectural concepts but refrains from claiming complete wire-format interoperability.
3. J. Postel, **"User Datagram Protocol"**, "RFC 768", "IETF", 1980. Used as the base datagram-format reference for the fixed-width application messages carried by the native Data Plane.
4. L. Eggert, G. Fairhurst, & G. Shepherd, **"UDP Usage Guidelines"**, "RFC 8085" / "BCP 145", "IETF", 2017. The current deployment is explicitly scoped as a controlled environment; general-Internet congestion-control & checksum guidance remains an applicability boundary.
5. W. Eddy, Ed., **"Transmission Control Protocol" ( "TCP" )**, "RFC 9293", "IETF", 2022. Used as the comparison reference for reliable byte-stream semantics, connection state, sequence / acknowledgment processing, & minimum header structure.
6. E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, **8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset**, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.
7. **"JPEG" Pleno Database**, *8i Voxelized Full Bodies ( 8iVFB v2 ) — A Dynamic Voxelized Point Cloud Dataset*.
8. **"DPDK" Project**, Data Plane Development Kit documentation, including Ethdev, "virtio-user", & "vhost-user" interfaces.
9. **NVIDIA**, "CUDA" Toolkit documentation & NVIDIA Video "Codec" / "NVENC" / hardware-decoding documentation.
10. **"FFmpeg" Project**, "FFmpeg" documentation, filter reference, hardware acceleration, & `vstats` facilities.
11. **Three.js Project**, Three.js / "WebGL" point-geometry documentation used by the browser presentation frontier.
12. **SciPy Project**, `scipy.spatial.cKDTree` documentation employed by the offline geometric `Gauge`.
13. Maria Giovanna Lacaria, **Point Cloud Coding for Extended Reality Services**, Master's Thesis, Sapienza University of Rome, Academic Year 2025/2026. This reference constitutes the application-level architectural & evaluation baseline explicitly discussed by the project.

---

## Final Note


The principal scholarly contribution of this repository resides in the **holistic co-design of packet transport, service steering, computation, reconstruction, control, presentation, orchestration granularity, & measurement** under a deliberately constrained software data-plane environment.

The point cloud is not merely relayed between isolated applications. `Camera` governs source admission from an `Encoder`-derived workload signal; `SFF1` computes useful geometry while packets traverse the data path; `SFF2` preserves experimental service state around two unaware application functions; `Encoder` projects & compresses the geometry through persistent "GPU" / "codec" state; `Decoder` reconstructs the point cloud & applies independent `User` "Pose"; `SFF3` closes the aware primary chain & classifies the reverse "Pose" path; & `User` separates native reception from asynchronous browser presentation while retaining explicit command & quality telemetry.

The use of "UDP" is therefore not justified by an undifferentiated claim that it is simply faster than "TCP". It follows from the current protocol semantics: fixed-size application entities are directly classifiable by SFFs, every control request encodes absolute latest state, duplicates / stale commands can be rejected deterministically, & unresolved state is re-presented until its application effect is observed in the returning data stream. This design avoids introducing byte-stream reassembly / connection-control state into the native "DPDK" service plane while retaining a clear reliability mechanism at the application frontier. Its validity is intentionally restricted to the documented controlled environment.

The retained evidence combines three repeated `B0` runtime executions, two complete capture-enabled fidelity controls ( `BQ` & `OQ-OFF` ), & the completed one-factor runtime sensitivity sweep. The repeated reference set preserves 300 / 300 frame completion, `current_skip = 1`, & approximately 30 frames / s; its interaction-rich `B0-R1` trace exercises 52 matched "Pose" directives, 51 of which acquire a positive browser `Command-to-Photon` sample before closure. The sensitivity runs separate source residency, geometry placement, point packetisation, & compressed-media grouping. The archived runtime `O-OFF` supplies a useful duplicated-work diagnostic in which local equivalent geometry recomputation activates "Temporal" while all 289 admitted frames remain complete; the fair `SFF1.NETWORK_PROCESSING_DISABLED + Encoder.OFFLOAD_MODE_DISABLED` reruns supersede it as the definitive runtime placement ablation. `P40` remains the packet-rate boundary that both activates source regulation & exposes final-hop Tx pressure.

Against the supplied probe-disabled application-level reference logs, the three `B0` Camera-to-User frame-ready medians are `211.352`, `214.176`, & `218.310 ms`; the best complete runtime trace is `B0-R1` at `211.352 ms` versus `341.785 ms` for Camera-to-Client reception in the reference. The corresponding measured reduction is `38.16 %`, with a run-level coefficient of variation of approximately `1.63 %`. The architecture assigns complementary roles to its three main mechanisms: "DPDK" supplies a uniform packet-processing execution model, "SFC" supplies explicit steering / control, & **in-network geometric processing removes reusable frame-preparation work from the Encoder boundary**. The final paired `SFF1` / `Encoder` ablation is the direct experiment for quantifying that in-network contribution.

Fidelity remains stable at the validated operating point. `BQ` reconstructs essentially the same mean final population as the supplied reference, & the fair capture-enabled `BQ` / `OQ-OFF` placement pair is exact at the `valid_points` vector: all 300 final populations match frame by frame. Mean `PSNR-Y` differs by only about `0.002 dB`, mean `SSIM-Y` remains near `0.997`, & the reported `Gauge` sequence means change by at most about `0.12 %`. This establishes that removing in-network geometry increases `Encoder` work without producing a material reconstructed-content shift at the measured quality resolution; the browser-enabled fair `O-OFF-R1 / R2` reruns quantify the corresponding runtime headroom / latency cost.

The orchestration contribution is similarly structural rather than rhetorical. The reference thesis identifies its monolithic `Encoder` & Client blocks as an obstacle to fine-grained placement & proposes their decomposition as future work. The current primary graph exposes seven native placement units rather than four coarse services — `75 %` more graph-level placement points — while the removal of one former switch-only "PMD" reservation makes `12.5 %` of this eight-thread host's logical scheduling capacity available for application placement. Neither percentage is presented as a measured reduction in total "CPU" consumption; the former quantifies granularity & the latter quantifies reallocation.

The repository has progressed beyond an upstream proof of concept into a complete experimental `Camera`-to-`User` platform whose remaining limitations are primarily methodological: the packet format is intentionally "NSH"-inspired rather than a generic interoperable stack, the custom six-view "HEVC" representation is not a standards-compliant point-cloud "codec", multi-user density remains unmeasured, & cross-reference resource percentages require a dedicated equal-hardware campaign. Runtime evidence covers the nominal non-overloaded "Temporal" state, the geometry-placement ablation, & elevated point-datagram pressure. `P40` now additionally exposes the complete workload growth / skip / recovery chronology, while the fair `O-OFF` reruns remove legacy duplicated geometry work & become the definitive in-network placement comparison. Broader overload / scalability envelopes remain future work.

The resulting platform is therefore best interpreted as an empirical study of **how a coarse application pipeline can be decomposed into explicit service functions, which volumetric-streaming operations benefit from in-path execution, how latest-state controls can remain reliable without importing a reliable byte stream into the packet path, how state can be retained around unaware functions, & how latency / fidelity / resource boundaries can be made sufficiently explicit for future orchestration decisions**.