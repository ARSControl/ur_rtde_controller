[33mcommit 9fb2f060c9fd772bdbd02ac434acf9864da80514[m[33m ([m[1;36mHEAD -> [m[1;32mapupa/devel[m[33m, [m[1;31morigin/apupa/devel[m[33m)[m
Author: Andrea Pupa <229867@studenti.unimore.it>
Date:   Sun Mar 19 19:38:37 2023 +0100

    added signal.h + format code

[33mcommit 9dffcf1efa01eeb62ef4a51e9a36f56df1e5f438[m[33m ([m[1;31morigin/master[m[33m, [m[1;31morigin/HEAD[m[33m, [m[1;32mmaster[m[33m)[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 16:30:40 2023 +0100

    Minor and Some TODO Bug to Fix

[33mcommit 57be177edf657fbd0fa311d178c66495e84128cc[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 16:06:10 2023 +0100

    Minor

[33mcommit a05f36052f51bba1a59971fbe9096b773892bf70[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 16:04:03 2023 +0100

    Minor

[33mcommit 2ea72cb9375199f71437e8e1adaf1edf5011eacf[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 15:47:56 2023 +0100

    Added Asyncronous Movement Flag and Functionality

[33mcommit 739cb3f0c33a9cb4b89ed78318dd9bd8531c34af[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 15:29:54 2023 +0100

    Readme Update

[33mcommit daf7695239599fe8441920deb02ce26861d6798a[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 15:29:12 2023 +0100

    Merged Position and Velocity Controllers

[33mcommit 7bbd94d2fa8aef598e10fa1cf9f25e03ff48e182[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 15:12:08 2023 +0100

    Added Eigen Utilities Functions

[33mcommit a7c4da8f1c1d1ae3213ae80b01dc39a6e1f71340[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 15:08:54 2023 +0100

    Added Joint and Cartesian Velocity Controllers
    
    Minor: Refactoring some code

[33mcommit 10481d0e2bf08d47ad73634ee79c32ac8ed10e68[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 12:59:24 2023 +0100

    Removed Trajectory Controller Until Fixing It

[33mcommit b6cbef72e90b1b0a10d82735ab24cbe4b08cce3b[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 17 12:56:46 2023 +0100

    Minor Refactoring

[33mcommit 5f413ab2ef07b41c0a6e87837da636c5fd9299b4[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 16 18:15:44 2023 +0100

    Bugfix
    
    Fixed Thread Closing
    Fixed Joint Position Controller

[33mcommit 9a625163c6614315f4d42d9bb942208b27330d4c[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 17:48:09 2023 +0100

    Removed useless `ros::spin0nce`
    
    WARNING: read callback in separate threads can cause unwanted behaviors

[33mcommit ed0a8c814887dc8cae2007eca0692d84cafb52c0[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 17:30:09 2023 +0100

    Fixes

[33mcommit a5d752d823f7979415e1cee6a787ef71321b4d6b[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 17:17:30 2023 +0100

    Bugfix: compute actual pose in main spinner

[33mcommit 87df03a3ca35a47d9bc4fd60b64642f175a69630[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 16:53:45 2023 +0100

    Bugfix

[33mcommit bc28eac765a5c79b59f2eef1f98dce32fa3aed00[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 16:50:59 2023 +0100

    Added `joint_states`, `cartesian_pose` and `FTSensor` publishers in separate threads

[33mcommit 7310f4feeb669180cf9b6e174a5a8e162133d53d[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 15:55:48 2023 +0100

    Readme Update

[33mcommit b110b6232b5d529b8efe77d3ff7de76f28bc7015[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 15:26:22 2023 +0100

    Fix

[33mcommit e262ec17e4a0030b5b1aa406a227a4c0c3c1da82[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 15:25:05 2023 +0100

    Added SIGINT

[33mcommit aa124aab78eebcf8d4a16a99f4d70fa01317f177[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 15:23:23 2023 +0100

    Create Gripper Service only if `enable_gripper` and create the new functions `Pose2RTDE` and `RTDE2Pose``

[33mcommit 45ccc6065ae0ba251e608b365189367454642643[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 12:35:34 2023 +0100

    Fixed Inverse Kinematic

[33mcommit fa87924f346ceba82fbbbca4f31315bd3dbdac14[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 12:28:09 2023 +0100

    BugFix ForwardKinematic Service

[33mcommit 62f9c70113ba98df592d64e71bb8307b506d83fa[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Wed Mar 15 11:28:39 2023 +0100

    Moved `RobotStatus` from Publisher to Service

[33mcommit 91db45cc78ec6cadc8a963556af8424bcbfb5573[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 17:41:40 2023 +0100

    Added FTSensor Readings Publisher

[33mcommit 2929ebf0e34b05b0abe0d35dacd9dc8b605bee88[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 17:24:54 2023 +0100

    Minor

[33mcommit 104fa87524ac6a149f294cf9a3ed991e1d2870a0[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 16:55:56 2023 +0100

    Changed `joint_states` topic + Cartesian Controller Fixes

[33mcommit e66f66a00a3fcad25590a119aff3db60f992c4b3[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 15:04:40 2023 +0100

    Changed `joint_states` topic

[33mcommit 80532efa799db7c0dc00be02a8373829c37669e1[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 15:03:52 2023 +0100

    Cleaning

[33mcommit 002395f2b74734dc2c9a33ed2df5de7d1e2ff414[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 14:19:06 2023 +0100

    Fixed `nh.param` reading, Changed `PoseStamped` with `Pose` | Fix jointGoalCallback

[33mcommit 2888bd8777c0a0db9c12578b5d1cd7522f28de03[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Mar 13 14:17:54 2023 +0100

    Fixed `nh.param` reading, Changed `PoseStamped` with `Pose`

[33mcommit 4edf89548a32434779f5b154e499f3b1a1cd7a26[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sun Mar 12 00:31:52 2023 +0100

    Bugfix

[33mcommit bdeccf3bf14189f28f93faff34b936c5bd6faae1[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sun Mar 12 00:23:52 2023 +0100

    Added Velocity Limits

[33mcommit f40b8eaaecb864709f443f55f5bb650a027c9031[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 11 23:55:49 2023 +0100

    Readme Update

[33mcommit 854944bfd05ddd58bd70f102e2cc6bc1531538c9[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 11 23:50:51 2023 +0100

    Added FK, IK, ZeroFT, FreedriveMode Services

[33mcommit dd3812a1f1ff54fd905998fc7e2d3307a5f6d477[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 11 22:53:01 2023 +0100

    Added Velocity Controller

[33mcommit c2bbec68a87062222b31a709cb29502b50431f55[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 11 22:47:06 2023 +0100

    Minor

[33mcommit c8d87590535c795c0395bf416e197475117326e2[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 16:24:46 2023 +0100

    Minor

[33mcommit 8d347e9aaa1998d814d1343e1bed70b0eb7b691d[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 16:04:53 2023 +0100

    Bugfixes and added Cartesian Controller

[33mcommit b779a11c4ae9d27fea752607afe39549e4c716f3[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 14:27:44 2023 +0100

    Rename `rtde_controller` in `position_controller`

[33mcommit f249d3d540ff9a3aa83647f5aeb03987e0406a00[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 14:23:52 2023 +0100

    Added RTDE Controller Node

[33mcommit 09445bee982309709206d9b9722dbb083019577b[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 14:18:56 2023 +0100

    Moved OLD Files in TODO Folder

[33mcommit 93a1e9055ec355a93c34c0956114e2b016c785e3[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Mar 10 14:16:56 2023 +0100

    Deleted ur_rtde submodule

[33mcommit c0050db20aea6ae19c351fcdeb7f0aa3eef5adab[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Jan 24 00:52:16 2023 +0100

    Minor

[33mcommit cf621b14eff27e5a1e17077cbb58ad9ab98d1053[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 19:17:31 2022 +0200

    little fixes

[33mcommit c92cdf18aad26e17d8e927698fba4f3630cfe51a[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 18:59:30 2022 +0200

    change saturation velocity computation

[33mcommit 488586dd10aa96261c148df6f45f55e1e24cfb86[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 18:52:37 2022 +0200

    up

[33mcommit 31230d0210ed339d9c32c96016254948609d8401[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 18:25:58 2022 +0200

    up

[33mcommit ed93386bbef2708afefd14cd950d694a4ba78d92[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 18:09:07 2022 +0200

    up

[33mcommit 149f6cc7abfdfb9e066b2ad4d085d4f94acdbe68[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 18:08:26 2022 +0200

    up

[33mcommit ab95d674810fd5b925990cf218a80471d4ba8146[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Tue Apr 5 17:21:28 2022 +0200

    changed stopRobot function

[33mcommit 099c280132987ad3f4c6e8db6277dae6a599794e[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Mon Apr 4 18:24:21 2022 +0200

    some changes

[33mcommit b4f5fea28314d953e77382e98cd7720d410b16d3[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Apr 1 01:42:35 2022 +0200

    added class destructor stop_robot

[33mcommit 3f387960d66ba2360b1475407437049935b80e8a[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Apr 1 01:33:26 2022 +0200

    added stop_robot service

[33mcommit 4faca0144c14a15e5b154c7363176ae5959dfc5d[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Fri Apr 1 00:46:53 2022 +0200

    convert cartesian_controller to class

[33mcommit 2ef1e7fb096acf6da592e17f24c9b966fa03f939[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 19:47:35 2022 +0200

    added "movement_precision" variable and service | added "trajectory_completed" publisher

[33mcommit af079ac9b851a1d44791bd6571113f1c879d5b1b[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 18:21:07 2022 +0200

    update

[33mcommit bc411fb636d5359d71f586e26d470e447f93d671[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 18:09:01 2022 +0200

    update

[33mcommit 7bd5807c7ae2098f28af06071ce29b5c4b5c843d[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 18:07:36 2022 +0200

    update

[33mcommit 97d0c53cbe786fb605bf536aecafd43a1cf1d668[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 18:01:19 2022 +0200

    update

[33mcommit 74c797b9de86307fa70905538cd21cfd158b15b7[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 16:41:01 2022 +0200

    update

[33mcommit b6fea1cf323d2a59f4beb267c6ef4140c991a351[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Thu Mar 31 14:56:44 2022 +0200

    class translation ur_speed_control

[33mcommit f87f05af7b75accdf4eb4e1536774b4048a5d344[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 26 16:02:44 2022 +0100

    update

[33mcommit 1eb96223aa9ffd5cea8c1c5c1d217fc926f3eea9[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 26 16:00:00 2022 +0100

    update readme

[33mcommit dc72620b9f6eb89b0c69a3e1078f5eeb99b94376[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 26 15:43:27 2022 +0100

    Launchfile + Parameters..

[33mcommit c39f7a5c722687b473d7279069f2ff2a337c5a47[m
Author: davideferrari95 <davideferr@unimore.it>
Date:   Sat Mar 26 14:24:23 2022 +0100

    minelli release

[33mcommit b3bcceac84c89e4d2909dbcafddab1093887167d[m
Author: Davide Ferrari <davide.ferrari95@unimore.it>
Date:   Sat Mar 26 13:20:28 2022 +0000

    Initial commit
