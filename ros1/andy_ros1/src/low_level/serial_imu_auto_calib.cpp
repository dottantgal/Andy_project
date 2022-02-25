/**
 *  @file       serial_imu_auto_calib.cpp
 *  @brief      Node to handle the auto calibration and flashing of the IMU teensy
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *  @author     Silvano Sallese <silvano.sallese@gmail.com>
 *
 *  @date       June 2018
 *  @copyright  Copyright (c) 2018 Hyperlync Technologies Inc, Orange Park,
 *              Florida U.S.A.
 *
 *              The Programs/Files (which include both the software and
 *              documentation) contain proprietary information of Hyperlync
 *              Technologies Inc; they are provided under a license agreement
 *              containing restrictions on use and disclosure and are also
 *              protected by copyright, patent, and other intellectual and
 *              industrial property laws.
 *
 *              Reverse engineering, disassembly, or decompilation of the
 *              Programs and Files is prohibited.
 */


#include <include/serial_sensor/serial_imu_auto_calib.h>


/* *** SeriaImuAutoCalib class *** */
/**
 * @brief   Node constructor.
 * @details Starts the serial imu autocalibration node.
 *          Subscribes to the /friday/fr_order ROS topic for receiving orders.
 */
SeriaImuAutoCalib::SeriaImuAutoCalib()
{
  ROS_INFO_STREAM( NODE_ID_ << "Starting the IMU autocalibration node");

  /* Parameters */
  int res = getParams();

  if (res != 0)
  {
    ROS_ERROR_STREAM(NODE_ID_ 
      << "Ops, cannot get correctly all the parameters!");
  }
  // -

  // subscribe to receive orders
  message_subscriber_ = nh_a_imu_.subscribe(
    "/friday/fr_order", 10, &SeriaImuAutoCalib::messageCallback, this);
  // -

}

/**
 * @brief SeriaImuAutoCalib class destructor.
 *
 */
SeriaImuAutoCalib::~SeriaImuAutoCalib()
{
  ROS_INFO_STREAM( NODE_ID_ << "Stopping the IMU autocalibration node");
}

/**
 * @brief Get the required ROS paramenters.
 * 
 * @return int 0 if all is ok. Negative number if something goes wrong.
 */
int SeriaImuAutoCalib::getParams()
{

  // enable/disable debug lines
  getParam("~debug", "Debug", &DEBUG_IMU_AUTOCALIB_, false, NODE_ID_, nullptr);

  // serial port name
  getParam("~port_name", "Serial port name", &PORT_NAME_PARAM_, "/dev/imu", NODE_ID_, nullptr);

  // database connection parameters
  getDBParams(&DB_HOST_, &DB_PORT_, &DB_USER_, &DB_PASS_, &DB_NAME_, NODE_ID_, nullptr);

  return 0;
}

/**
 * @brief   Callback to subscribe to the /friday/fr_order ROS topic.
 * @details If the order_id part of the NodeOrder ROS message is 15
 *          then checks the type_task value:
 *            - if its value is 1 then starts the autocalibration task
 *            - if its value is 3 then the autocalibration is stopped
 *
 * @param   msg Needed to access to the NodeOrder ROS message.
 */
void SeriaImuAutoCalib::messageCallback(const hyperlync_ros::NodeOrder::ConstPtr &msg)
{
  if ( msg->order_id == 15 )
  {
    ROS_DEBUG_STREAM( NODE_ID_ << 
      "IMU autocalibration node received type_task=" << msg->type_task
      << " order_id=" << msg->order_id
      << " name=" << msg->order_argument);

    switch(msg->type_task)
    {
      case 1: //Run the autocalibration
      {
        ROS_INFO_STREAM(NODE_ID_ << "Starting autocalibration...");

        runAutocalibration();

        exit(0);

        break;
      }

      case 2: //Run the only reboot
      {
        ROS_INFO_STREAM(NODE_ID_ << "Starting reboot...");

        runOnlyReboot();

        break;
      }

      case 3: //Run the soft reboot
      {
        ROS_INFO_STREAM(NODE_ID_ << "Starting soft reboot...");

        runSoftReboot();

        break;
      }

      case 4: //Run the hard reboot
      {
        ROS_INFO_STREAM(NODE_ID_ << "Starting hard reboot...");

        runHardReboot();

        break;
      }

      // case 3:
      // {
      //   ROS_INFO_STREAM(NODE_ID_ << "Stopping...");

      //   break;
      // }
      default: {}
    }
  }
}


/**
 * @brief   Function to update the IMU autocalibration progress in the DB
 * 
 */
void SeriaImuAutoCalib::updateProgressDb(int percentage)
{
  op_status_ = FridayDatabaseAPI::setImuAutocalibProgress(
      percentage, DB_HOST_, DB_PORT_, DB_USER_, DB_PASS_, DB_NAME_);
  if (op_status_.second.compare("true") != 0)
    ROS_WARN_STREAM(NODE_ID_ << "Ops, something goes wrong: " << op_status_.first);
}

/**
 * @brief   Function to perform the reboot of the Teensy board
 * 
 */
void SeriaImuAutoCalib::runOnlyReboot()
{

	if (! teensyOpen()) { 
		ROS_WARN_STREAM("Could not find HalfKay");
	} exit(0);
	ROS_INFO_STREAM("Found Teensy HalfKay Bootloader\n");
  unsigned char buf_reboot[2048];
  int block_size_reboot = 512;
	boot(buf_reboot, block_size_reboot+2);
	exit(0);

}

/**
 * @brief   Function to perform the soft reboot of the Teensy board
 * 
 */
void SeriaImuAutoCalib::runSoftReboot()
{
  usb_dev_handle *serial_handle = NULL;

	serial_handle = openUsbDevice(0x16C0, 0x0483);
	if (!serial_handle) {
		char *error = usb_strerror();
		ROS_WARN_STREAM("Error opening USB device: " << error << "\n");
		exit(0);
	}

	char reboot_command = 134;
	int response = usb_control_msg(serial_handle, 0x21, 0x20, 0, 0, &reboot_command, 1, 10000);

	usb_release_interface(serial_handle, 0);
	usb_close(serial_handle);

	if (response < 0) {
		char *error = usb_strerror();
		ROS_WARN_STREAM("Unable to soft reboot with USB error: " << error << "\n");
		exit(0);
	}
  
  ROS_INFO_STREAM("Soft reboot performed\n");

}


/**
 * @brief   Function to perform the hard reboot of the Teensy board
 * 
 */
void SeriaImuAutoCalib::runHardReboot()
{
  usb_dev_handle *rebootor;
	int response;

	rebootor = openUsbDevice(0x16C0, 0x0477);
	if (!rebootor) exit(0);
	response = usb_control_msg(rebootor, 0x21, 9, 0x0200, 0, (char *)"reboot", 6, 100);
	usb_release_interface(rebootor, 0);
	usb_close(rebootor);
	if (response < 0) {
		char *error = usb_strerror();
		ROS_WARN_STREAM("Unable to hard reboot with USB error: " << error << "\n");
		exit(0);
	}
  
  ROS_INFO_STREAM("Hard reboot performed\n");
}


/**
 * @brief   Function to start the IMU autocalibration task
 * 
 */
void SeriaImuAutoCalib::runAutocalibration()
{
  HOME_PATH = getenv("HOME");

  ros::Rate r(10);

  // /* ** Old method : Compile and upload through arduino avr ** */
  // sys_call = system("arduino --upload --board teensy:avr:teensyLC --port /dev/imu ~/hyperlync_ros_data/imu_fw/MPU6050calibration/MPU6050calibration.ino > /dev/null &");
  // if ( sys_call < 0 ) {
  //     ROS_WARN_STREAM("Ops, something goes wrong uploading the calibration FW: ");
  // } else {
  //   ROS_INFO_STREAM("The calibration FW is being loaded....please wait");
  //   ros::Duration(15).sleep();
  // }
  // /* *** */

  updateProgressDb(1);

  /* ** Compiling of the calibration FW through arduino avr ** */
  compiling_calib_fw: 
	  sys_call = system("arduino --verify --board teensy:avr:teensyLC ~/hyperlync_ros_data/imu_fw/MPU6050calibration/MPU6050calibration.ino > /dev/null &");
	  if ( sys_call < 0 ) {
	      ROS_WARN_STREAM( NODE_ID_ << "Ops, something goes wrong compiling the calibration FW!!!");
	  } else {
	    ROS_INFO_STREAM( NODE_ID_ << "The calibration FW is being compiled....please wait");
	    ros::Duration(25).sleep();
	  }
    /* *** */

    updateProgressDb(5);

	  // killing of the teensy GUI process
	  killTeensyGui();

	  /* ** Uploading of the firware through the teensyloader api ** */
    end_record_seen = 0;
    extended_addr = 0;
	  unsigned char buf[2048];
	  int code_size = 63488, block_size = 512; // These are the main variable to identify the TeensyLC
		int num, addr, write_size=block_size+2;
	  int return_flash;
		int first_block=1, waited=0;

	  // set this to 1 to enable just the boot
	  // int boot_only = 0;

	  // use hard reboot if device not online
	  //int hard_reboot_device = 0;

	  // to wait or not for the device to appear
	  int wait_for_device_to_appear = 0;

	  // to enable or not a soft reboot
	  int soft_reboot_device = 1;

	  // to enable or not the reboot after the flashing
	  int reboot_after_programming = 1;

		// boot only and exit: COMMENTED, probably we'll never use it
		// if (boot_only) {
		// 	if (! teensy_open()) { //it's performed using the usb.h library of libusb
		// 		ROS_WARN_STREAM("Could not find HalfKay");
		// 	}
		// 	ROS_INFO_STREAM("Found Teensy HalfKay Bootloader\n");

		// 	boot(buf, block_size+2);
		// 	exit(0);
		// }

		// read the hex file created by the avr compiler
	  std::string hex_file = HOME_PATH;
	  hex_file = hex_file.append("/hyperlync_ros_data/imu_fw/imu_hex/MPU6050calibration.ino.hex");
	  num = readIntelHex(hex_file.c_str());
	  if (num < 0) {
      ROS_WARN_STREAM( NODE_ID_ << "Error reading hex firmware file : " << hex_file.c_str());
      goto compiling_calib_fw;
      }
	  ROS_INFO_STREAM( NODE_ID_ << "Read " << hex_file << ": " << num << "bytes, " << 
      (double)num / (double)code_size * 100.0 << " usage\n");
    
    updateProgressDb(10);  

    // open the USB device
    while (1) {
      if (teensyOpen()) break;
      // if (hard_reboot_device) {
      //   if (!hardReboot()) ROS_WARN_STREAM( NODE_ID_ << "Unable to find rebootor\n");
      //   ROS_INFO_STREAM( NODE_ID_ << "Hard Reboot performed\n");
      //   hard_reboot_device = 0; // only hard reboot once
      //   wait_for_device_to_appear = 1;
      // }
      if (soft_reboot_device) {
        runSoftReboot();
        soft_reboot_device = 0;
        wait_for_device_to_appear = 1;
      }
      if (!wait_for_device_to_appear) ROS_WARN_STREAM( NODE_ID_ << "Unable to open device\n");
      if (!waited) {
        ROS_INFO_STREAM( NODE_ID_ << "Waiting for Teensy device...\n");
        ROS_INFO_STREAM( NODE_ID_ << " (hint: press the reset button)\n");
        waited = 1;
      }
    usleep(0.25 * 1000000.0);
    }
    ROS_INFO_STREAM( NODE_ID_ << "Found HalfKay Bootloader\n");

    updateProgressDb(12);
    
    // reading again the hex file to be sure that isnt changed
    if (waited) {
      num = readIntelHex(hex_file.c_str());
      if (num < 0) {
        ROS_WARN_STREAM( NODE_ID_ << "Error reading hex firmware file : " << hex_file.c_str());
        goto compiling_calib_fw;
        }
      ROS_INFO_STREAM( NODE_ID_ << "Read " << hex_file << ": " << num << "bytes, " << 
        (double)num / (double)code_size * 100.0 << " usage\n");
    }

    updateProgressDb(15);
    
	  // programming the board with the new firmware
    ROS_INFO_STREAM( NODE_ID_ << "Flashing the calibration firmware, please wait....");
    fflush(stdout);
    for (addr = 0; addr < code_size; addr += block_size) {
      if (!first_block && !ihexBytesWithinRange(addr, addr + block_size - 1)) {
        // don't waste time on blocks that are unused,
        // but always do the first one to erase the chip
        continue;
      }
      if (!first_block && memoryIsBlank(addr, block_size)) continue;
      ROS_INFO_STREAM( NODE_ID_ << ".");
      if (block_size <= 256 && code_size < 0x10000) {
        buf[0] = addr & 255;
        buf[1] = (addr >> 8) & 255;
        ihexGetData(addr, block_size, buf + 2);
        write_size = block_size + 2;
      } else if (block_size == 256) {
        buf[0] = (addr >> 8) & 255;
        buf[1] = (addr >> 16) & 255;
        ihexGetData(addr, block_size, buf + 2);
        write_size = block_size + 2;
      } else if (block_size == 512 || block_size == 1024) {
        buf[0] = addr & 255;
        buf[1] = (addr >> 8) & 255;
        buf[2] = (addr >> 16) & 255;
        memset(buf + 3, 0, 61);
        ihexGetData(addr, block_size, buf + 64);
        write_size = block_size + 64;
      } else {
        ROS_WARN_STREAM( NODE_ID_ << "Unknown code/block size\n");
      }
      return_flash = teensyWrite(buf, write_size, first_block ? 5.0 : 0.5);
      if (!return_flash) {
        ROS_INFO_STREAM( NODE_ID_ << "error writing to Teensy\n");
        runSoftReboot();
        goto compiling_calib_fw;
        }
      first_block = 0;
    }
    ROS_INFO_STREAM( NODE_ID_ << "\n");

    // reboot to the user's new code
    if (reboot_after_programming) {
      boot(buf, write_size);
    }
    teensyClose(); 
    ROS_INFO_STREAM( NODE_ID_ << "Calibration firmware flashed on Teensy");  
    /* *** */

    updateProgressDb(20);

    ros::Duration(5).sleep();

    /* ** File writing of the serial output coming from the Teensy ** */
    port = PORT_NAME_PARAM_;
    std::string end_string ("done");
    calibration_str = HOME_PATH;
    calibration_str = calibration_str.append("/hyperlync_ros_data/calibration_imu.txt");
    calibration_file.open(calibration_str);

    int count_serial = 0;

    while(ros::ok())
    {
      try
      {
        if (ser.isOpen())
        {
          // read strings from Teensy
          if(ser.available())
          {
            read = ser.read(ser.available());
            ROS_INFO_STREAM( NODE_ID_ << "read " << (int)read.size() << 
              "new characters from serial port, adding to " << (int)input.size() <<
              " characters of old input." );
            input += read;
            count_serial ++;
            if (count_serial == 20){
              updateProgressDb(30);
            } else if (count_serial == 30){
              updateProgressDb(35);
            } else if (count_serial == 40){
              updateProgressDb(40);
            } else if (count_serial == 50){
              updateProgressDb(45);
            } else if (count_serial == 60){
              updateProgressDb(50);
            } else if (count_serial == 70){
              updateProgressDb(60);
            } else if (count_serial == 80){
              updateProgressDb(70);
            }
            
            // filling of the calibration_imu.txt file
            calibration_file << read << std::endl;
            std::size_t found = read.find(end_string);
            if (found!=std::string::npos) {
              ROS_INFO_STREAM( NODE_ID_ << "Closing the calibration file");
              calibration_file.close();
              break;
            }
          //-
          }
        }
        else
        {
          // try and open the serial port
          try
          {
            ser.setPort(port);
            ser.setBaudrate(57600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
          }
          catch (serial::IOException& e)
          {
            ROS_INFO_STREAM( NODE_ID_ << "Unable to open serial port " << ser.getPort()
              << ". Trying again in 5 seconds.");
            ros::Duration(5).sleep();
          }

          if(ser.isOpen())
          {
            ROS_INFO_STREAM( NODE_ID_ << "Serial port " << ser.getPort()
              << " initialized and opened.");
          }
        }
      }
      catch (serial::IOException& e)
      {
        ROS_INFO_STREAM( NODE_ID_ << "Error reading from the serial port " << ser.getPort()
          << ". Closing connection.");
        ser.close();
      }
      ros::spinOnce();
      r.sleep();
    }
    ser.close();
    /* *** */

    updateProgressDb(75);

    /* ** Searching "done" string position in the generated calibration_imu.txt ** */
    inFile.open(calibration_str);

    if(!inFile){
      ROS_WARN_STREAM( NODE_ID_ << "Ops, something goes wrong opening the calibration file!!!");
      exit(1);
    }

    size_t pos;
    int line_number = 1;
    while(inFile.good()){
      getline(inFile,line);
      pos = line.find(end_string);
      if(pos!=std::string::npos)
      {
        //ROS_INFO_STREAM("FOUND AT LINE = " << line_number << " AND POSITION = " << pos);
        inFile.close();
        break;
      } else line_number ++;
    }
    /* *** */

    /* ** Get the line with the calibration values ** */
    std::string calibration_values;
    inFile.open(calibration_str);
    for (int lineno = 1; getline (inFile,line) && lineno < line_number; lineno++){
      if (lineno == (line_number - 1) ){
        calibration_values = line;
        //ROS_INFO_STREAM("CALIBRATION VALUES LINE \n" << calibration_values);   
      }
    }
    inFile.close();
    /* *** */

    /* ** Get the single calibration values ** */
    std::string str2 = "[";
    size_t pos_a = 0;
    size_t pos_space = 0;
    int count_space = 0;

    unsigned first;
    unsigned last;
    std::string strNew;

    while ((pos_space = calibration_values.find(str2, pos_a)) < calibration_values.length()) 
    {
      pos_a = pos_space + str2.length();
      if (count_space == 0){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        XAccel = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "First calibration value extracted = " << strNew); 
      }
      if (count_space == 2){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        YAccel = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "Second calibration value extracted = " << strNew); 
      }
      if (count_space == 4){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        ZAccel = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "Third calibration value extracted = " << strNew); 
      }
      if (count_space == 6){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        XGyro = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "Fourth calibration value extracted = " << strNew); 
      }
      if (count_space == 8){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        YGyro = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "Fifth calibration value extracted = " << strNew); 
      }
      if (count_space == 10){
        first = calibration_values.find_first_of("[", pos_a - 1);
        last = calibration_values.find_first_of(",", pos_a);
        strNew = calibration_values.substr (first + 1,last-(first + 1));
        ZGyro = strNew;
        ROS_INFO_STREAM( NODE_ID_ << "Sixth calibration value extracted = " << strNew); 
      }
      
      count_space ++;
    } 
    /* *** */

    updateProgressDb(80);

    /* ** Creation of the Teensy firmware with the calibration values ** */
    ROS_INFO_STREAM( NODE_ID_ << "Creating the Teensy ROSLIB firmware....."); 
    std::string firmware_in_str = HOME_PATH;
    firmware_in_str = firmware_in_str.append("/hyperlync_ros_data/imu_fw/MPU6050roslib/MPU6050roslib.ino");
    std::string firmware_out_str = HOME_PATH;
    firmware_out_str = firmware_out_str.append("/hyperlync_ros_data/imu_fw/MPU6050roslib/MPU6050roslibCalib/MPU6050roslibCalib.ino");

    std::string strOld_XAccel = "accelgyro.setXAccelOffset();";
    std::string strOld_YAccel = "accelgyro.setYAccelOffset();";
    std::string strOld_ZAccel = "accelgyro.setZAccelOffset();";
    std::string strOld_XGyro = "accelgyro.setXGyroOffset();";
    std::string strOld_YGyro = "accelgyro.setYGyroOffset();";
    std::string strOld_ZGyro = "accelgyro.setZGyroOffset();";

    std::string strNew_XAccel = "accelgyro.setXAccelOffset(";
    strNew_XAccel.append(XAccel);
    strNew_XAccel.append(");");
    std::string strNew_YAccel = "accelgyro.setYAccelOffset(";
    strNew_YAccel.append(YAccel);
    strNew_YAccel.append(");");
    std::string strNew_ZAccel = "accelgyro.setZAccelOffset(";
    strNew_ZAccel.append(ZAccel);
    strNew_ZAccel.append(");");
    std::string strNew_XGyro = "accelgyro.setXGyroOffset(";
    strNew_XGyro.append(XGyro);
    strNew_XGyro.append(");");
    std::string strNew_YGyro = "accelgyro.setYGyroOffset(";
    strNew_YGyro.append(YGyro);
    strNew_YGyro.append(");");
    std::string strNew_ZGyro = "accelgyro.setZGyroOffset(";
    strNew_ZGyro.append(ZGyro);
    strNew_ZGyro.append(");");

    std::ifstream filein(firmware_in_str); // file to read from
    std::ofstream fileout(firmware_out_str); // file to write on

    pos = 0; // reset the seek position

    if(!filein || !fileout)
    {
      ROS_WARN_STREAM( NODE_ID_ << "Ops, something goes wrong opening the file!!!");
      //return 1;
    }

    while(filein.good()){
      getline(filein,line); // get line from file
      pos = line.find(strOld_XAccel);
      if(pos!=std::string::npos)
      {
        fileout << "\t" << strNew_XAccel << "\n";
      } else {
        pos = line.find(strOld_YAccel);
        if(pos!=std::string::npos)
        {
          fileout << "\t" << strNew_YAccel << "\n";
        } else {
          pos = line.find(strOld_ZAccel);
          if(pos!=std::string::npos)
          {
            fileout << "\t" << strNew_ZAccel << "\n";
          } else {
            pos = line.find(strOld_XGyro);
            if(pos!=std::string::npos)
            {
              fileout << "\t" << strNew_XGyro << "\n";
            } else {
              pos = line.find(strOld_YGyro);
              if(pos!=std::string::npos)
              {
                fileout << "\t" << strNew_YGyro << "\n";
              } else {
                pos = line.find(strOld_ZGyro);
                if(pos!=std::string::npos)
                {
                  fileout << "\t" << strNew_ZGyro << "\n";
                } else fileout << line << "\n";
                }
              }
          }
        }
      }
    }
    filein.close();
    fileout.close();
    /* *** */

    updateProgressDb(85);

    // /* ** OLD METHOD : Uploading of the IMU ros_lib firmware previously created ** */
    // ros::Duration(5).sleep();

    // sys_call = system("arduino --upload --board teensy:avr:teensyLC --port /dev/imu ~/hyperlync_ros_data/imu_fw/MPU6050roslib/MPU6050roslibCalib/MPU6050roslibCalib.ino > /dev/null &");
    // if ( sys_call < 0 ) {
    //     ROS_WARN_STREAM("Ops, something goes wrong uploading the calibration FW: ");
    // } else {
    //   ROS_INFO_STREAM("The IMU firmware is being loaded....please wait");
    //   ros::Duration(15).sleep();
    // }
    // /* *** */
  compiling_ros_lib:
    /* ** Compiling through arduino avr ** */
    sys_call = system("arduino --verify --board teensy:avr:teensyLC ~/hyperlync_ros_data/imu_fw/MPU6050roslib/MPU6050roslibCalib/MPU6050roslibCalib.ino > /dev/null &");
    if ( sys_call < 0 ) {
        ROS_WARN_STREAM( NODE_ID_ << "Ops, something goes wrong compiling the ROSLIB FW!");
    } else {
      ROS_INFO_STREAM( NODE_ID_ << "The ROSLIB FW is being compiled....please wait");
      ros::Duration(35).sleep();
    }
    /* *** */

    updateProgressDb(90);

    // killing of the teensy GUI process
    killTeensyGui();
    ros::Duration(5).sleep();

    /* ** Uploading of the firware through the teensyloader api ** */
    code_size = 63488;
    block_size = 512; // These are the main variable to identify the TeensyLC
    write_size=block_size+2;
    first_block=1;
    waited=0;

    // use hard reboot if device not online
    //hard_reboot_device = 0;

    // to wait or not for the device to appear
    wait_for_device_to_appear = 0;

    // to enable or not a soft reboot
    soft_reboot_device = 1;

    // to enable or not the reboot after the flashing
    reboot_after_programming = 1;

    // read the hex file created by the avr compiler
    std::string hex_roslib_file = HOME_PATH;
    hex_roslib_file = hex_roslib_file.append("/hyperlync_ros_data/imu_fw/imu_hex/MPU6050roslibCalib.ino.hex");
    num = readIntelHex(hex_roslib_file.c_str());
    if (num < 0) {
      ROS_WARN_STREAM( NODE_ID_ << "Error reading hex firmware file :" << hex_roslib_file.c_str());
      goto compiling_ros_lib;
      }
    ROS_INFO_STREAM( NODE_ID_ << "Read " << hex_roslib_file << ": " << num << "bytes, " << 
      (double)num / (double)code_size * 100.0 << " usage\n");
    
    updateProgressDb(92);

    // open the USB device
    while (1) {
      if (teensyOpen()) break;
      // if (hard_reboot_device) {
      //   if (!hardReboot()) ROS_WARN_STREAM( NODE_ID_ << "Unable to find rebootor\n");
      //   ROS_INFO_STREAM( NODE_ID_ << "Hard Reboot performed\n");
      //   hard_reboot_device = 0; // only hard reboot once
      //   wait_for_device_to_appear = 1;
      // }
      if (soft_reboot_device) {
        runSoftReboot();
        soft_reboot_device = 0;
        wait_for_device_to_appear = 1;
      }
      if (!wait_for_device_to_appear) ROS_WARN_STREAM( NODE_ID_ << "Unable to open device\n");
      if (!waited) {
        ROS_INFO_STREAM( NODE_ID_ << "Waiting for Teensy device...\n");
        ROS_INFO_STREAM( NODE_ID_ << " (hint: press the reset button)\n");
        waited = 1;
      }
      usleep(0.25 * 1000000.0);
    }
    ROS_INFO_STREAM( NODE_ID_ << "Found HalfKay Bootloader\n");

    updateProgressDb(95);

    // if we waited for the device, read the hex file again
    // perhaps it changed while we were waiting?
    if (waited) {
      num = readIntelHex(hex_roslib_file.c_str());
      if (num < 0) {
        ROS_WARN_STREAM( NODE_ID_ << "Error reading hex firmware file :" << hex_roslib_file.c_str());
        goto compiling_ros_lib;
        }
      ROS_INFO_STREAM( NODE_ID_ << "Read " << hex_roslib_file << ": " << num << "bytes, " << 
        (double)num / (double)code_size * 100.0 << " usage\n");
    }

    // programming the board with the new firmware
      ROS_INFO_STREAM( NODE_ID_ << "Flashing the ROSLIB firmware, please wait....");
      fflush(stdout);
      for (addr = 0; addr < code_size; addr += block_size) {
        if (!first_block && !ihexBytesWithinRange(addr, addr + block_size - 1)) {
          // don't waste time on blocks that are unused,
          // but always do the first one to erase the chip
          continue;
        }
        if (!first_block && memoryIsBlank(addr, block_size)) continue;
        ROS_INFO_STREAM( NODE_ID_ << ".");
        if (block_size <= 256 && code_size < 0x10000) {
          buf[0] = addr & 255;
          buf[1] = (addr >> 8) & 255;
          ihexGetData(addr, block_size, buf + 2);
          write_size = block_size + 2;
        } else if (block_size == 256) {
          buf[0] = (addr >> 8) & 255;
          buf[1] = (addr >> 16) & 255;
          ihexGetData(addr, block_size, buf + 2);
          write_size = block_size + 2;
        } else if (block_size == 512 || block_size == 1024) {
          buf[0] = addr & 255;
          buf[1] = (addr >> 8) & 255;
          buf[2] = (addr >> 16) & 255;
          memset(buf + 3, 0, 61);
          ihexGetData(addr, block_size, buf + 64);
          write_size = block_size + 64;
        } else {
          ROS_WARN_STREAM( NODE_ID_ << "Unknown code/block size\n");
        }
        return_flash = teensyWrite(buf, write_size, first_block ? 5.0 : 0.5);
        if (!return_flash) {
          ROS_INFO_STREAM( NODE_ID_ << "Error writing to Teensy\n");
          runSoftReboot();
          goto compiling_ros_lib;
          }
        first_block = 0;
      }
      ROS_INFO_STREAM( NODE_ID_ << "\n");

      // reboot to the user's new code
      if (reboot_after_programming) {
        boot(buf, write_size);
      }
      teensyClose(); 

      ROS_INFO_STREAM( NODE_ID_ << "ROSLIB firmware flashed on Teensy"); 
      //ros::shutdown();
      updateProgressDb(100);
    /* *** */
}

/**
 * @brief   Method to capture the stdout useful to make the calibration values files
 * 
 */
std::string SeriaImuAutoCalib::captureStdout(std::string cmd) {

  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream) {
  while (!feof(stream))
  if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
  pclose(stream);
  }
  return data;
}
/* *** */


/**
 * @brief   TEENSY_LOADER : Method to open the usb device
 * 
 */
usb_dev_handle * SeriaImuAutoCalib::openUsbDevice(int vid, int pid)
{
	struct usb_bus *bus;
	struct usb_device *dev;
  usb_dev_handle *dev_handler;
	char buf[128];
  int usb_op_res;

	usb_init();
	usb_find_busses();
	usb_find_devices();
	//printf_verbose("\nSearching for USB device:\n");
	for (bus = usb_get_busses(); bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			//printf_verbose("bus \"%s\", device \"%s\" vid=%04X, pid=%04X\n",
			//	bus->dirname, dev->filename,
			//	dev->descriptor.idVendor,
			//	dev->descriptor.idProduct
			//);
			if (dev->descriptor.idVendor != vid) continue;
			if (dev->descriptor.idProduct != pid) continue;
			dev_handler = usb_open(dev);
			if (!dev_handler) {
				ROS_WARN_STREAM( NODE_ID_ << "Found device but unable to open\n");
				continue;
			}
			usb_op_res = usb_get_driver_np(dev_handler, 0, buf, sizeof(buf));
			if (usb_op_res >= 0) {
				usb_op_res = usb_detach_kernel_driver_np(dev_handler, 0);
				if (usb_op_res < 0) {
					usb_close(dev_handler);
					ROS_WARN_STREAM( NODE_ID_ << "Device is in use by " << buf << " driver\n");
					continue;
				}
			}
			usb_op_res = usb_claim_interface(dev_handler, 0);
			if (usb_op_res < 0) {
				usb_close(dev_handler);
				ROS_WARN_STREAM( NODE_ID_ << "Unable to claim interface, check USB permissions\n");
				continue;
			}
      
			return dev_handler;
		}
	}
	return NULL;
}


/**
 * @brief   TEENSY_LOADER : Closing of the teensy usb connection 
 * 
 */
void SeriaImuAutoCalib::teensyClose(void)
{
	if (!libusb_teensy_handle) return;
	usb_release_interface(libusb_teensy_handle, 0);
	usb_close(libusb_teensy_handle);
	libusb_teensy_handle = NULL;
}
/* *** */


/**
 * @brief   TEENSY_LOADER : Opening of the teensy usb connection 
 * 
 */
int SeriaImuAutoCalib::teensyOpen(void)
{
	teensyClose();
	libusb_teensy_handle = openUsbDevice(0x16C0, 0x0478);
	if (libusb_teensy_handle) return 1;
	return 0;
}


/**
 * @brief   TEENSY_LOADER : Writing/flashing of the teensy memory
 * 
 */
int SeriaImuAutoCalib::teensyWrite(void *buf, int len, double timeout)
{
	int write_res;

	if (!libusb_teensy_handle) return 0;
	while (timeout > 0) {
		write_res = usb_control_msg(libusb_teensy_handle, 0x21, 9, 0x0200, 0,
			(char *)buf, len, (int)(timeout * 1000.0));
		if (write_res >= 0) return 1;
		//printf("teensy_write, r=%d\n", r);
		usleep(10000);
		timeout -= 0.01;  // TODO: subtract actual elapsed time
	}
	return 0;
}


// /* ** TEENSY_LOADER : Method to handle the writing errors ** */
// COMMENTED CAUSE IT'S NOT USED AT THE MOMENT
// void die(const char *str, ...)
// {
// 	va_list  ap;

// 	va_start(ap, str);
// 	vfprintf(stderr, str, ap);
// 	fprintf(stderr, "\n");
// 	exit(1);
// }
// /* *** */


/**
 * @brief   TEENSY_LOADER : Method to softly boot the teensy
 * 
 */
void SeriaImuAutoCalib::boot(unsigned char *buf, int write_size)
{
	ROS_INFO_STREAM( NODE_ID_ << "Booting\n");
	memset(buf, 0, write_size);
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	teensyWrite(buf, write_size, 0.5);
}


/**
 * @brief   TEENSY_LOADER : Method which parse the hex file for the upload
 * 
 */
int SeriaImuAutoCalib::parseHexLine(char *line)
{
	int addr, code, num;
  int sum, len, cksum, i;
  char *ptr;

  num = 0;
  if (line[0] != ':') return 0;
  if (strlen(line) < 11) return 0;
  ptr = line+1;
  if (!sscanf(ptr, "%02x", &len)) return 0;
  ptr += 2;
  if ((int)strlen(line) < (11 + (len * 2)) ) return 0;
  if (!sscanf(ptr, "%04x", &addr)) return 0;
  ptr += 4;
    /* printf("Line: length=%d Addr=%d\n", len, addr); */
  if (!sscanf(ptr, "%02x", &code)) return 0;
	if (addr + extended_addr + len >= MAX_MEMORY_SIZE) return 0;
  ptr += 2;
  sum = (len & 255) + ((addr >> 8) & 255) + (addr & 255) + (code & 255);
	if (code != 0) {
		if (code == 1) {
			end_record_seen = 1;
			return 1;
		}
		if (code == 2 && len == 2) {
			if (!sscanf(ptr, "%04x", &i)) return 1;
			ptr += 4;
			sum += ((i >> 8) & 255) + (i & 255);
        		if (!sscanf(ptr, "%02x", &cksum)) return 1;
			if (((sum & 255) + (cksum & 255)) & 255) return 1;
			extended_addr = i << 4;
			//printf("ext addr = %05X\n", extended_addr);
		}
		if (code == 4 && len == 2) {
			if (!sscanf(ptr, "%04x", &i)) return 1;
			ptr += 4;
			sum += ((i >> 8) & 255) + (i & 255);
        		if (!sscanf(ptr, "%02x", &cksum)) return 1;
			if (((sum & 255) + (cksum & 255)) & 255) return 1;
			extended_addr = i << 16;
			//printf("ext addr = %08X\n", extended_addr);
		}
		return 1;	// non-data line
	}
	byte_count += len;
  while (num != len) {
    if (sscanf(ptr, "%02x", &i) != 1) return 0;
		i &= 255;
		firmware_image[addr + extended_addr + num] = i;
		firmware_mask[addr + extended_addr + num] = 1;
    ptr += 2;
    sum += i;
    (num)++;
    if (num >= 256) return 0;
  }
  if (!sscanf(ptr, "%02x", &cksum)) return 0;
  if (((sum & 255) + (cksum & 255)) & 255) return 0; /* checksum error */
  return 1;
}


/**
 * @brief   TEENSY_LOADER : Method to check the hex file if the chosen flash method is the teensy loader
 * 
 */
int SeriaImuAutoCalib::readIntelHex(const char *filename)
{
	FILE *fp;
	int i, lineno=0;
	char buf[1024];

	byte_count = 0;
	end_record_seen = 0;
	for (i=0; i<MAX_MEMORY_SIZE; i++) {
		firmware_image[i] = 0xFF;
		firmware_mask[i] = 0;
	}
	extended_addr = 0;

	fp = fopen(filename, "r");
	if (fp == NULL) {
		//printf("Unable to read file %s\n", filename);
    ROS_WARN_STREAM( NODE_ID_ << "Unable to read file " << filename << "\n");
		return -1;
	}
	while (!feof(fp)) {
		*buf = '\0';
		if (!fgets(buf, sizeof(buf), fp)) break;
		lineno++;
		if (*buf) {
			if (parseHexLine(buf) == 0) {
				ROS_INFO_STREAM( NODE_ID_ << "Warning, HEX parse error line " << lineno << "\n");
				return -2;
			}
		}
		if (end_record_seen) break;
		if (feof(stdin)) break;
	}
	fclose(fp);
	return byte_count;
}


/**
 * @brief   TEENSY_LOADER : Method to know if the hex bytes are in range
 * 
 */
int SeriaImuAutoCalib::ihexBytesWithinRange(int begin, int end)
{
	int i;

	if (begin < 0 || begin >= MAX_MEMORY_SIZE ||
	   end < 0 || end >= MAX_MEMORY_SIZE) {
		return 0;
	}
	for (i=begin; i<=end; i++) {
		if (firmware_mask[i]) return 1;
	}
	return 0;
}


/**
 * @brief   TEENSY_LOADER : Method to know if the teensy memory is blank
 * 
 */
int SeriaImuAutoCalib::memoryIsBlank(int addr, int block_size)
{
	if (addr < 0 || addr > MAX_MEMORY_SIZE) return 1;

	while (block_size && addr < MAX_MEMORY_SIZE) {
		if (firmware_mask[addr] && firmware_image[addr] != 255) return 0;
		addr++;
		block_size--;
	}
	return 1;
}


/**
 * @brief   TEENSY_LOADER : Method to get the data from the hex file
 * 
 */
void SeriaImuAutoCalib::ihexGetData(int addr, int len, unsigned char *bytes)
{
	int i;

	if (addr < 0 || len < 0 || addr + len >= MAX_MEMORY_SIZE) {
		for (i=0; i<len; i++) {
			bytes[i] = 255;
		}
		return;
	}
	for (i=0; i<len; i++) {
		if (firmware_mask[addr]) {
			bytes[i] = firmware_image[addr];
		} else {
			bytes[i] = 255;
		}
		addr++;
	}
}

/**
 * @brief   Method to kill the Teensy GUI
 * 
 */
void SeriaImuAutoCalib::killTeensyGui(){
  int sys_call;
  sys_call = system("ps axf | grep teensy | grep -v grep | "
          "awk '{print \"kill -s 15 \" $1}' | sh > /dev/null");
  if ( sys_call < 0 ) {
      ROS_WARN_STREAM( NODE_ID_ << "Ops, something goes wrong killing the Teensy GUI!!");
  } else {
    ROS_INFO_STREAM( NODE_ID_ << "Teensy GUI killed");
  }
}

/**
 * @brief   Main function
 * @details Creates the IMU autocalibration node named 'hyperlync_ros_imu_autocalib_node'
 *
 */
int main(int argc, char **argv) {

  ros::init(argc, argv, "hyperlync_ros_imu_autocalib_node");

  SeriaImuAutoCalib imu_autocalib_node;

  ros::spin();
}
