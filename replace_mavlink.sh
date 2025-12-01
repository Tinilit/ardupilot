#!/bin/bash
set -e

echo "=== Removing old MAVLink submodule ==="
git submodule deinit -f modules/mavlink || true
git rm -f modules/mavlink || true
rm -rf .git/modules/modules/mavlink || true

echo "=== Adding new MAVLink submodule ==="
git submodule add https://github.com/Tinilit/mavlink.git modules/mavlink

echo "=== Switching to branch: my-plane-xmls ==="
cd modules/mavlink
git checkout my-plane-xmls
cd ../..

echo "=== Staging changes ==="
git add .gitmodules modules/mavlink

echo "=== Done ==="
