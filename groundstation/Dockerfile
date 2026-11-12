# DroneCtrl ground station.
#
# Two stages: a full Go toolchain to build a static binary, then distroless
# static, which has no shell, no package manager and runs as a non-root user.
# The service needs only the standard library, so there is nothing to fetch at
# build time beyond the base images.
#
#   docker build -t dronectrl-groundstation groundstation/
#   docker run -p 8080:8080 -p 14550:14550/udp dronectrl-groundstation

FROM golang:1.24-alpine AS build
WORKDIR /src
COPY go.mod ./
COPY . .
# CGO off for a static binary; -trimpath and stripped symbols for a small,
# reproducible one.
RUN CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/groundstation ./cmd/groundstation

FROM gcr.io/distroless/static-debian12:nonroot
COPY --from=build /out/groundstation /groundstation
# The SIL telemetry logs, so the image can demo itself with -replay.
COPY testdata/ /testdata/
EXPOSE 8080/tcp 14550/udp
USER nonroot:nonroot
ENTRYPOINT ["/groundstation"]
CMD ["-udp", ":14550", "-http", ":8080"]
