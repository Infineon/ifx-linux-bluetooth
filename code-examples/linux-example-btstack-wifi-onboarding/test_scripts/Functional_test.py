import paramiko
import time
import threading
import select
from scp import SCPClient

device_connected = 0

def findme_service_handler(logfile, execTimeout):
    global device_connected
    advert_start = "Advertisement started"
    connect_msg = "Connected"
    ip='10.3.89.69'
    port=22
    name='ifx'
    password='ifx'
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=ip,username=name,password=password)   #This is used to establish a connection
    remote_connection = ssh_client.invoke_shell() #This helps you invoke the shell of the client machine
    remote_connection.send("sudo ./cyw5557x/scripts/bt_autobaud.sh\n")
    remote_connection.send(password +"\n")
    time.sleep(10)
    
    counter = execTimeout * 5
    remote_connection.send("cd Executables\n")     #to the remote machine that you are trying to connect with
    remote_connection.send("./linux-findme -c /dev/ttyTHS1 -b 3000000 -d 112233221133 -i 1 -f 921600 -p /home/ifx/cyw5557x/BT/CYW55560A1_001.002.087/fcbga_iPA_sLNA_ANT0/CYW55560A1_001.002.087.0058.0000_Generic_UART_37_4MHz_fcbga_iPA_sLNA_ANT0.hcd\n")

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
    ssh_client.close
    
def RemoveExtraLine(logs):
    c = logs.count("\n")
    for i in range(c):
        logs.remove("\n")

def GetAlertLevel(line):
     start_index = line.rfind("=")
     start_index = start_index+1
     value = int(line[start_index:start_index+1])
     return value
     
def verify_logs(logfile):
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


def initial_setup():
    ip='10.3.89.69'
    port=22
    name='ifx'
    password='ifx'
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname=ip,username=name,password=password)   #This is used to establish a connection
    scp = SCPClient(ssh_client.get_transport())
    scp.put('Executables', recursive=True, remote_path='/home/ifx/')
    scp.close()
    ssh_client.close
    
#    ip='10.3.89.91'
#    port=22
#    name='ifx'
#    password='ifx'
#    ssh_client = paramiko.SSHClient()
#    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#    ssh_client.connect(hostname=ip,username=name,password=password)   #This is used to establish a connection
#    scp = SCPClient(ssh_client.get_transport())
#    scp.put('Executables', recursive=True, remote_path='/home/ifx/')
#    scp.close()
#    ssh_client.close


def run_test():
    #This thread will handle GATT server commands and logs. Pass log file name in argument
    t1 = threading.Thread(target=findme_service_handler, args=('Find_Me_service.txt',100))
    
    #This thread will handle GATT client commands and logs. Pass log file name along with total time of execution in arguments
#    t2 = threading.Thread(target=gatt_client_hanler, args=('GATT_Client.txt',100,))
  
    # starting thread 1
    t1.start()
    
    time.sleep(10) #Allow time for server to setup ADV parameters
    
    # starting thread 2
#    t2.start()
  
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

