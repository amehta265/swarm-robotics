"""
Phase 3 — Swarm communication protocol with pheromone grid patches.

Message schema (JSON):
{
  "robot_id":   "walnut" | "hazel",
  "timestamp":  float,          # sim time when this was sent
  "position":   [x, y, z],      # sender's world position
  "heading":    float,          # sender's yaw in radians
  "patch": {                    # pheromone grid patch
    "timestamp": float,         # when patch was built (for staleness)
    "cells":     [              # sparse list of informative cells
      {"wx": float, "wy": float, "mu": float, "sigma2": float},
      ...
    ],
    "n_cells": int
  }
}

Why sparse patches and not the full grid?
──────────────────────────────────────────
A 60×60 grid = 3600 cells × (wx,wy,mu,sigma2) ≈ 115KB per message at 10Hz
= 1.15MB/s per robot. Over real radio this is too much.

Sparse patches (only cells where mu > threshold) typically contain
10-50 cells = ~2-4KB per message. Bandwidth drops 30-100x.
This is why real swarm systems use sparse representations.

No shared memory: each robot owns its grid privately.
All synchronization happens through these UDP messages.
"""
import json
import socket
import time

WALNUT_PORT = 5100
HAZEL_PORT  = 5101
HOST        = '127.0.0.1'
MSG_TIMEOUT = 0.001   # 1ms — non-blocking


def build_message(robot_id, timestamp, position, heading, patch):
    return {
        'robot_id':  robot_id,
        'timestamp': round(timestamp, 4),
        'position':  [round(p, 4) for p in position],
        'heading':   round(heading, 4),
        'patch':     patch,
    }


def encode(msg):
    return json.dumps(msg).encode('utf-8')

def decode(data):
    return json.loads(data.decode('utf-8'))


def make_sender():
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def make_receiver(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, port))
    sock.settimeout(MSG_TIMEOUT)
    return sock

def send(sock, msg, dest_port):
    try:
        data = encode(msg)
        # UDP max payload ~65KB — warn if patch is large
        if len(data) > 32768:
            print(f"  WARNING: message is {len(data)} bytes — consider"
                  f" raising mu_thresh to reduce patch size")
        sock.sendto(data, (HOST, dest_port))
    except Exception:
        pass   # fire and forget — UDP doesn't guarantee delivery

def recv(sock):
    """
    Non-blocking receive. Returns decoded message or None.
    None means no message arrived this timestep — not an error.
    Real radio comms drop packets all the time; agents must be robust to this.
    """
    try:
        data, _ = sock.recvfrom(65535)
        return decode(data)
    except socket.timeout:
        return None
    except Exception:
        return None