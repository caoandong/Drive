#!/bin/bash
pid=$(pgrep xboxdrv)
sudo kill -9 $pid
