FROM rust:latest AS builder

WORKDIR /app

# Build deps for reqwest (native-tls) + sqlite
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    libssl-dev \
    libsqlite3-dev \
    ca-certificates \
    clang \
    && rm -rf /var/lib/apt/lists/*

COPY . .

RUN cargo build -p atc-server --bin atc-server --release

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libssl3 \
    libsqlite3-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN mkdir -p /app/data

COPY --from=builder /app/target/release/atc-server /usr/local/bin/atc-server

ENV RUST_LOG=info
EXPOSE 3000

ENTRYPOINT ["atc-server"]
