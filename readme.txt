1/在线程中手动运行回调函数时，需要将回调函数的形参由geometry::Twist &xxx改为geomestry::Twist::ConstPtr &xxx，不然就会提示引用错误invalid initialization of reference type

2/运行map_server时：
错误1：terminate called after throwing an instance of 'ros::InvalidNameException'
  what():  Character [.] at element [96] is not valid in Graph Resource Name [/home/lab307/zgd_turtlebot/src/turtlebot/src/turtlebot_simulator/turtlebot_gazebo/maps/playgound.yaml].  Valid characters are a-z, A-Z, 0-9, / and _.
解决：map_file:mymap.yaml写成了map_file:=mymap.yaml多加了一个=

错误2：[ERROR] [1497583668.219121078]: Map_server could not open map_file:/home/lab307/zgd_turtlebot/src/turtlebot/src/turtlebot_simulator/turtlebot_gazebo/maps/playground.yaml.
解决：待解决


3/运行gmapping完储存地图map_saver时
错误1：[ERROR] [1497582911.711563957, 837.910000000]: Couldn't save map file to /home/lab307/zgd_turtlebot/my_map/new.pgm
解决：


4/dynamic reconfig
错误1：cmakelist文件中，写完.cfg文件不用添加依赖项然后编译，需要将用到config文件的cpp写完，然后添加executable后编译，所谓的自动生成compositeConfig.h文件并不是显示生成，所以在include文件夹里找不到，但可以编译通过

5/relocation 0 has invalid symbol index 11
cpp文件中main函数要写在namespace{}之外

6/

