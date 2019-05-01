#!/usr/bin/env bash

kill -9 $(ps aux | grep '[v]rserver' | grep -v grep | awk '{print $2}')
