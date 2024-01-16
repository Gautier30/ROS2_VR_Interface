#!/bin/sh

node ./server.js &
node ./robotPoseSubscriber.js &
node ./cmdVelPublisher_1.js &
node ./cmdVelPublisher_2.js &
node ./cmdVelPublisher_3.js &
node ./goalPosePublisher_1.js &
node ./goalPosePublisher_2.js &
node ./goalPosePublisher_3.js &
node ./camSubscriber_1.js &
node ./camSubscriber_2.js &
node ./camSubscriber_3.js &
