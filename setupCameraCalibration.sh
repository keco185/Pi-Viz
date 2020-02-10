#!/bin/bash

v4l2-ctl --set-ctrl auto_exposure=1
v4l2-ctl --set-ctrl white_balance_auto_preset=1
v4l2-ctl --set-ctrl iso_sensitivity_auto=1
v4l2-ctl --set-ctrl power_line_frequency=0
