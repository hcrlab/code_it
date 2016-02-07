#!/bin/sh
. ~/.nvm/nvm.sh
nvm use 0.10.40
cd ../build/bundle
export ROOT_URL='http://localhost'
export PORT=3003
node main.js
