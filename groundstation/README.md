# DroneCtrl ground station

A small Go service that receives DroneCtrl's MAVLink telemetry and serves it:
a live dashboard, JSON, Server-Sent Events, Prometheus metrics, and health
probes for Kubernetes. Standard library only, so it builds anywhere with
nothing to fetch.

```
go run ./cmd/groundstation -udp :14550 -http :8080
```

Then open http://localhost:8080. Point a vehicle (or MAVProxy forwarding a
serial link) at UDP 14550.

## Without a vehicle

Every SIL scenario writes the telemetry the board would have sent as a
`.tlog`. Replay one at real speed:

```
make -C ../firmware/sil run
go run ./cmd/groundstation -replay ../firmware/sil/logs/signal_loss.tlog -loop
```

The dashboard then shows the signal-loss flight: armed, hovering, the link
lost, `FAILSAFE_LAND`, and the cut once the descent ramp reaches zero, with
the state changes listed.

`-check` replays as fast as possible, prints what it decoded as JSON, and
exits non-zero if any frame was rejected, any sequence number was skipped, or
no heartbeat arrived. CI runs it over every SIL log.

## Endpoints

| Path | What |
| --- | --- |
| `/` | live dashboard |
| `/api/vehicles`, `/api/vehicles/{id}` | vehicle state as JSON |
| `/api/link` | parser counters: frames, CRC errors, sequence gaps, unknown messages |
| `/api/stream` | the vehicle list as Server-Sent Events, 5 Hz |
| `/metrics` | Prometheus text format |
| `/healthz` | the process answers |
| `/readyz` | the telemetry input is up (UDP bound, or a replay started) |

Metrics include connection, armed and failsafe flags, a one-hot
`dronectrl_vehicle_flight_state{state=...}`, attitude, battery, the control
loop's load, jitter and missed deadlines as the board reports them, and the
link's frame, CRC error and sequence gap counters.

## Container

```
docker build -t dronectrl-groundstation .
docker run -p 8080:8080 -p 14550:14550/udp dronectrl-groundstation
```

Two stages: the Go toolchain builds a static binary, which runs in distroless
static as a non-root user with no shell. The image carries the SIL logs, so
`-replay /testdata/signal_loss.tlog -loop` works out of the box.

## Kubernetes

```
kubectl apply -k deploy/k8s/base           # listen for vehicles
kubectl apply -k deploy/k8s/overlays/demo  # loop the SIL signal-loss flight
kubectl -n dronectrl port-forward svc/groundstation 8080:80
```

- **One replica, `Recreate` strategy.** State is in memory and UDP has no
  session to pin a vehicle to a pod; two replicas would each see half the
  frames and both report a lossy link.
- **Restricted Pod Security Standard**, enforced on the namespace: non-root,
  read-only root filesystem, no privilege escalation, all capabilities
  dropped, RuntimeDefault seccomp, no service account token.
- **Separate Services** for the dashboard (ClusterIP) and the MAVLink input
  (NodePort 31455/UDP, `externalTrafficPolicy: Local` so the sender's address
  survives), so the telemetry port can be exposed without exposing the
  dashboard with it.
- **Readiness means the input is up**, not just that the process answers, so
  a pod whose UDP port did not bind never receives traffic.

The manifests are validated against the Kubernetes 1.30 schemas with
kubeconform, and CI deploys the demo overlay to a kind cluster, waits for the
rollout and checks the service reports a connected vehicle.

The image reference is `ghcr.io/khalednoaman/dronectrl-groundstation`; it is
not published yet, so build and load it (or push it) before applying the base
outside CI.

## Layout

| Package | What |
| --- | --- |
| `internal/mavlink` | v2 codec, the third implementation in the repository; tests decode firmware-written logs and compare against pymavlink bytes |
| `internal/tlog` | QGroundControl telemetry log reader |
| `internal/vehicle` | per-vehicle state, heartbeat timeout, state change history |
| `internal/server` | HTTP, SSE, Prometheus exposition, the dashboard |
| `cmd/groundstation` | flags, UDP input, replay, graceful shutdown |
