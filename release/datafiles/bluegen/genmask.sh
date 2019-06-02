#!/bin/sh
if [ ! -d node_modules ]; then
  npm update
fi

./node_modules/.bin/bluegen -c config.json

