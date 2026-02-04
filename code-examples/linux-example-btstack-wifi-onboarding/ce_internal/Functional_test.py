import paramiko
import time
import threading
import select
import xml.etree.ElementTree as ET
import sys
import os
from scp import SCPClient
sys.path.insert(1, os.environ['CI_PROJECT_DIR'] + "/ci/linux_train")
from ft_flash_executable import *

device_connected = 0

def wifiOnboarding_service_handler(logfile, execTimeout):
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

    advert_start = "Advertisement Started"
    connect_msg = "Connected"
    port=22

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=ip1,username=name1,password=password1)   #This is used to establish a connection
    remote_connection = ssh_client.invoke_shell() #This helps you invoke the shell of the client machine
    
    counter = execTimeout * 5
    remote_connection.send("cd Executables\n")     #to the remote machine that you are trying to connect with
    remote_connection.send("sudo chmod +x *\n")
    remote_connection.send(password1 + "\n")
    time.sleep(10)
    flash_cmds = []

    flash_cmds = flashCode(sys.argv[1],sys.argv[2], sys.argv[3], sys.argv[10])
    for i in range(0, len(flash_cmds)):
        if (flash_cmds[i][0] == 0):
            continue
        else:
            command = flash_cmds[i][0]
            print(command)
            remote_connection.send(str(command))
            remote_connection.send("\n")
            time.sleep(flash_cmds[i][1])
    time.sleep(10)
    print("Starting WiFi-Onboarding CE now...\n")
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
    #remote_connection.send("rm -r Executables\n")
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
    wifionboarding_log_file = open(logfile, 'r')
    
    wifionboarding_logs = wifionboarding_log_file.readlines()
    RemoveExtraLine(wifionboarding_logs)
 	    
    if (device_connected == 1):
        print("Client and Server connected successfully")
        for i in range(0, len(wifionboarding_logs)):
            line = wifionboarding_logs[i]
            line = line.replace(" ", "")
            if ("Wi-FiSSID" in line):
                ssid = line.partition(":")[2]
                print("Recieved SSID: ", ssid)
    else:
        print("Client not found. Exiting test")
     	

def run_test():
    '''
    This API is used to start the functional test of CE.
    2 individual threads are started for Client and Server execution.
    
    Input: None
    
    Output: None
    '''
    
    global ip1, ip2, name1, name2, password1, password2
    global logfile
    ip1 = sys.argv[4]
    name1 = sys.argv[5]
    password1 = sys.argv[6]

    ip2 = sys.argv[7]
    name2 = sys.argv[8]
    password2 = sys.argv[9]

    hcdfile = sys.argv[10]
    hcdtype=hcdfile.partition("iPA")[2].split(".")[0]

    logfile = "WifiOnboarding" + "_" + sys.argv[1] + "_" + sys.argv[2] + hcdtype + ".txt"
    #This thread will handle GATT server commands and logs. Pass log file name in argument
    t1 = threading.Thread(target=wifiOnboarding_service_handler, args=(logfile,200))
    
 
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
    global logfile
    run_test()

    verify_logs(logfile)
