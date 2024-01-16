#!/bin/bash

xterm -e "node ./server.js; bash" &

xterm -e "node ./robotPoseSubscriber.js; bash" &

xterm -e "node ./cmdVelPublisher_1.js; bash" &
xterm -e "node ./goalPosePublisher_1.js; bash" &
xterm -e "node ./camSubscriber_1.js; bash" &


xterm -e "node ./cmdVelPublisher_2.js; bash" &
xterm -e "node ./goalPosePublisher_2.js; bash" &
xterm -e "node ./camSubscriber_2.js; bash" &


xterm -e "node ./cmdVelPublisher_3.js; bash" &
xterm -e "node ./goalPosePublisher_3.js; bash" &
xterm -e "node ./camSubscriber_3.js; bash" &
