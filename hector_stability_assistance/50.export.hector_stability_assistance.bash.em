#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_USE_STABILITY_ASSISTANCE "ON=1,OFF=0" "Additionally start hector_stability_assistance for stability-related assistance functions."
fi
