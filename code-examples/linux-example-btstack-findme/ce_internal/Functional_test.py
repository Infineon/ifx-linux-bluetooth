import paramiko
import time
import threading
import select
import xml.etree.ElementTree as ET
import sys
from scp import SCPClient

device_connected = 0

def findme_service_handler(logfile, execTimeout):
    '''
    This API handles the findme CE execution.
         1. Connects to the board using SSH.
         2. Flash the SO file
         3. Wait for Client side board to be up
         4. If Client gets connected, wait for alert notification.
         5. Save SSH logs in log file.
         6. Repeat step 5 until client closes the connection
         
         Input: logfile - TXT file to which SSH logs are to be written
         
         Output: None
    '''
    global device_connected
    global ip1, ip2, name1, name2, password1, password2

    advert_start = "Advertisement started"
    connect_msg = "Connected"
    port=22

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=ip1,username=name1,password=password1)   #This is used to establish a connection
    remote_connection = ssh_client.invoke_shell() #This helps you invoke the shell of the client machine
#    remote_connection.send("sudo ./cyw5557x/scripts/bt_autobaud.sh\n")
#    remote_connection.send(password +"\n")
#    time.sleep(10)
    
    counter = execTimeout * 5
    remote_connection.send("cd Executables\n")     #to the remote machine that you are trying to connect with
    remote_connection.send("sudo chmod +x *\n")
    remote_connection.send(password1 + "\n")
    time.sleep(10)
    remote_connection.send("./linux-findme -c /dev/ttyTHS1 -b 3000000 -d 112233221133 -u /dev/gpiochip0 190 -r /dev/gpiochip1 16 -i 1 -f 921600 -p /home/ifx/cyw5557x/BT/CYW55560A1_001.002.087/fcbga_iPA_sLNA_ANT0/CYW55560A1_001.002.087.0058.0000_Generic_UART_37_4MHz_fcbga_iPA_sLNA_ANT0.hcd\n")

    time.sleep(10)
    print("Starting Find Me service now...\n")
    file = open(logfile, 'w')
    remote_connection.setblocking(0)
    while True:
        counter = counter - 1
        if (counter == 0):
            break
        backMsg = ""
        ready = select.select([remote_connection], [], [], 0.2)
        if ready[0]:
            backMsg = remote_connection.recv(65536)
        if len(backMsg) != 0:
            backMsg = backMsg.decode('utf-8')
            if (advert_start in backMsg):
            	print("Advertisement started. Waiting for client")
            if (connect_msg in backMsg):
            	print("Client Connected")
            	device_connected = 1
            #print (backMsg)
            file.write(backMsg)
    #print("Server Execution finished\n")
    file.close()
    remote_connection.send("\x03\n")    
    remote_connection.send("cd ..\n")
    remote_connection.send("rm -r Executables\n")
    time.sleep(10)
    ssh_client.close
    
def RemoveExtraLine(logs):
    ''' This API is used to remove the unwanted newlines characters in log file.
    This makes parsing log files faster.
    
    Input: logs - log file from the \n to be removed
    
    Output: none. Log file is altered in place.
    '''
    c = logs.count("\n")
    for i in range(c):
        logs.remove("\n")
        
def GetAlertLevel(line):
     '''
     This API is used to extract the alert value from the specfic line in the log file.
     Alert level can be 0 (No alert), 1 (Medium Alert), 2 (High Alert). The logs contains "Alert Level = <0/1/2>"
          
     Input: The log line from which alert level need to be extracted.
     
     Output: Alertlevel
     '''
     start_index = line.rfind("=")
     start_index = start_index+1
     value = int(line[start_index:start_index+1])
     return value


def verify_logs(logfile):
    ''' 
    This API is used to verify the SSH logs collected during CE execution. The logs are parsed for the presence/absence of specific messages from CE.
    CE displays appropriate messages as the CE execution advances. For eg., it displays "Advertisement started" when the BT advertisements start. Similary, when 
    "FindMe" device gets connected, "Connected" message is displayed. 
    
    Input: logfile - FindMe log file
    
    Output: If client is connected during CE execution and alert levels are modified, the new alert level is displayed. Else, client not fond message is displayed.
    ''' 
    global device_connected
#    advert_start = "Advertisement started"
#    connect_msg = "Connected"
#    device_connected = 0
    findme_log_file = open(logfile, 'r')
    
    findme_logs = findme_log_file.readlines()
    RemoveExtraLine(findme_logs)
 	    
    if (device_connected == 1):
        print("Client and Server connected successfully")
        for i in range(0, len(findme_logs)):
            line = findme_logs[i]
            line = line.replace(" ", "")
            if ("AlertLevel" in line):
            	alert_level = GetAlertLevel(line)
            	print("Alert level changed to ", alert_level)
    else:
        print("Client not found. Exiting test")
     	
def GetBoardInfo():
    '''
    This API is used to get the board configuration data such as IP Address, username and password from Board_Config.xml to connect to the board.
    The TARGET and RADIO information required is obtained as command-line arguments.
    
    Input: sys.argv[1] - TARGET (could be XAVIER/NANO/RPi etc)
    sys.argv[2] - RADIO (could be HATCHET2/54591 etc)
    
    Ouput:
    ip1 - IP address of first board
    name1 - Username of first board
    password1 - Password of first board
    
    ip2 - IP address of second board
    name2 - Username of second board
    password2 - Password of second board
    
    '''
    global ip1, ip2, name1, password1, name2, password2
    tree = ET.parse('//home//test//Board_Config//Board_Config.xml')
    root = tree.getroot()

    for setup in root.findall('TEST_SETUP'):
        target = setup.find('TARGET').text
        radio = setup.find('RADIO').text
        if ((target == sys.argv[1]) & (radio == sys.argv[2])):
            ip1 = setup.find('IP1').text
            ip2 = setup.find('IP2').text
            name1 = setup.find('USERNAME1').text
            password1 = setup.find('PASSWORD1').text
            name2 = setup.find('USERNAME2').text
            password2 = setup.find('PASSWORD2').text

def initial_setup():
    '''
    This API is used to premilinary setups before executing the CE.
    1. Retreive IP adress, username and password and connect to board
    2. Copy the CE executable, dependency SO files to the target board.
    
    Input: None
    
    Output: None
    '''
    global ip1, ip2, name1, name2, password1, password2
    GetBoardInfo()
    port=22
    
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=ip1,username=name1,password=password1)   #This is used to establish a connection
    scp_client = SCPClient(ssh_client.get_transport())
    scp_client.put('Executables', recursive=True, remote_path='/home/ifx')
    scp_client.close()
    ssh_client.close


def run_test():
    '''
    This API is used to start the functional test of CE.
    2 individual threads are started for Client and Server execution.
    
    Input: None
    
    Output: None
    '''
    
    #This thread will handle GATT server commands and logs. Pass log file name in argument
    t1 = threading.Thread(target=findme_service_handler, args=('Find_Me_service.txt',100))
    
 
    # starting thread 1
    t1.start()
    
    time.sleep(10) #Allow time for server to setup ADV parameters
    
     # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
#    t2.join()
  
    # both threads completely executed
    #print("\n**************Done!**************\n")

if __name__ == "__main__":
    initial_setup()
    run_test()

    verify_logs("Find_Me_service.txt")

