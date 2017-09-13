#!/bin/bash

# vcloud only
vc_frame_id="cloud"
vc_child_frame_id="cloud"

# others frame here


fid=${vc_frame_id}
cid=${vc_child_frame_id}

echo "rosrun tf static_transform_publisher 0 0 0 0 0 0 1 ${fid} ${cid}"

rosrun tf static_transform_publisher 0 0 0 0 0 0 1 ${fid} ${cid}
