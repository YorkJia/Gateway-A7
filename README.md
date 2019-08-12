	This project is develop a cloud Gateway, which can support 
a variety of drivers,collect field data, then push the data to the 
cloud.At present, it only supports modbus rtu driver and aliyun.

# compile the sqlite3 for arm process

1. Extract the sqlite3 src to sqlite3-arm

2. mkdir install.   in /opt/sqlite3-arm

3. ./configure --host=arm-linux-gnueabihf --prefix=/opt/sqlite3-arm/install

4. make

5. make install

6. copy the /install/bin/sqlite3 to Armboard/usr/local/bin

7. copy the /install/lib/sqlite3.so.0.8.6 to Armboard/usr/local/lib

8. copy the /install/include/sqlite3 to our project.

# /********** log **************/

# 05311730:

(1) the project can running ok, in arm board.

(2) the pthread func can not work.

(3) the sqlite3 operator ok.
	
(4) we can read the cfg.db->dev_table ok.

(5) there are many warning compile.

(6) transmit paras to thread is the key.

# 06041011:
(1) the project compile has no warning.

# 06041409:
(1) modify the struct, not use point(*), but use arrary,malloc size.
    then can tx para to thread ok.

# 07162238:
(1) add the driver folder,and mb_rtu driver thread, test well.
(2) remote the sqlite for the test success.

# 07201551:
(1) we add the libmodbus into proj,and the libmodbus test ok.

# 07201921:
(1) modify the thread para transfer mode: transfer level 1 pointer to the driver thread,
    but the pointer consist by on struct,and the struct const by level 2 pointer.
    so,we can use one driver thread,scanning many slave device,and many section data.

# 07202211:
(1) modify the thread para transfer mode: transfer level 1 pointer,this is more simple
	than level2 pointer. we need to know, the level 1 pointer can include lot of
	data, becase,pointer is addr,the addr can be start addr. start addr! start addr!
	behind the pointer, the more data the more you want. 

# 07211638:
(1) for our thread can both read and write,we use the event fsm,and now,the program can running ok.

# 07212152:
(1) modify the thread para transfer mode, we define a type,include void * pointer, this mode is 
	good for driver extension,now the test can running ok.

# 07222227:
(1) add the modbus write regs func, and the function is running ok.
(2) we use the gettimeofday() get the tick of OS for count the delay time,function is ok,but the
    write func may miss sometimes. function ok.

# 07232306:
(1) we modify the program design,use general mb_rtu driver thread, test ok,must have bug.
(2) modify the driver thread para type.
