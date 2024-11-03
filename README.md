# proc
proc_v1 的源码


ide使用arduino， 型号选择  arduino uno  
把板子上的串口接到电脑上，就可以编译下载了，

用命令行进行编译:  
执行build.sh, 会使用arduino-cli进行编译下载。如果没有arduino-cli,会自动下载arduino-cli, 如果缺库， 脚本会自动下载库， 完成编译后， rom会在build.sh所在的目录prc.hex

用命令行进行升级:
把设备接到usb串口上，如果使用被控机进行升级， 根据串口设备编号，修改脚本，然后，linux下要退出串口login程序对串口的占用。   
执行update.sh， 然后给在倒计时到1时， 给设备上电， 升级过程就会开始。
如果升级失败，调整一下给设备上电的时机，再试一下。 
