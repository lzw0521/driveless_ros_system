chanpwd(){
  cd ~/driveless_ros_system_0903
  source devel/setup.bash
  cd src/launch
}

closebash(){
   var=` ps -ef | grep ros | grep -v grep | awk '{print $2}' `
   if [ "$var" != "" ];then
		kill -9 $var
                echo 'closed'
   else	
		echo 'ok'

   fi
}


if [[ $1 == '-stop' ]]; then
   closebash
   
fi

if [[ $1 == '-start' ]]; then
   gnome-terminal -x bash -c "roslaunch rosbridge_server rosbridge_websocket.launch ;exec bash;"
   sleep 1
   chanpwd
   gnome-terminal -x bash -c "roslaunch launch/planner.launch ;exec bash;"
   sleep 1
   chanpwd
   gnome-terminal -x bash -c "roslaunch launch/can.launch;exec bash;"
   sleep 1
   chanpwd
   gnome-terminal -x bash -c "roslaunch launch/gps.launch;exec bash;"
   
fi


#sleep 1
#chanpwd
#gnome-terminal -x bash -c "roslaunch ultrasonics.launch;exec bash;"

