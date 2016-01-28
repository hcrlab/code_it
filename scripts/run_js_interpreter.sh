#!/bin/sh
cd ../build/bundle
export ROOT_URL='http://localhost'
export PORT=3003
node main.js
