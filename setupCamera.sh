#!/bin/bash
raspistill --nopreview -md 7 -o startupView.jpg -w 640 -h 480 -ss 600000
v4l2-ctl --set-ctrl auto_exposure=1
v4l2-ctl --set-ctrl white_balance_auto_preset=1
v4l2-ctl --set-ctrl iso_sensitivity_auto=0
v4l2-ctl --set-ctrl power_line_frequency=0
