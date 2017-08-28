#!/bin/sh
export NVM_DIR="/var/ros/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
nvm use 0.10.40
cd ../build/bundle
export ROOT_URL='http://localhost'
export PORT=3003
node -v
node main.js
