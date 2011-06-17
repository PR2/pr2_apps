#!/usr/bin/env python


# enable debugging
import cgi, cgitb
import os
import pexpect
import time
cgitb.enable()

import popen2


print "Content-type: text/html"
print

print "RESULT"

form = cgi.FieldStorage()
message = form.getvalue("action", "NO_ACTION")

def run(fun):
    out = ""
    v = popen2.popen4(fun)
    for i in v[0]:
        out = out + i
    return out

def run_as_robot(command):
    run = "su applications -c \"" + command + "\""
    print "C", run
    child = pexpect.spawn(run)
    result = child.expect(["ssword:", "(yes/no)?"])
    child.sendline("willow")
    out = ""
    for i in child.readlines():
        out = out + i
    return out



if (message == "GET_STATE"):
    print "USERS"
    active_user = "(UNKNOWN)"
    dead_users = ""
    for i in run("robot users").split("\n"):
        if (i.find("Active User:") != -1):
            active_user = i.split(":")[1].strip()
        if (i.find("*") != -1):
            dead_users = dead_users + i[i.find("*") + 1:i.find("(")].strip() + ","


    print "ACTIVE_USER: ", active_user
    print "INACTIVE_USERS:", dead_users.strip().strip(",")

    print "PROCESSES:"
    processes = ""
    for i in run_as_robot("robot plist").split("\n"):
        if (i.strip() != "Password:" and i.strip() != "The following processes are running:" and i.strip() != "" and i.strip() != "No processes running."):
            l = i.strip().split(None)
            if (len(l) > 5):
                print l[5], "from", l[4]
            else:
                print "INVALID:", i.strip()

    print
elif (message == "STOP_ROBOT"):
    print "STOPPING_ROBOT"
    result = run_as_robot("yes | robot claim ; yes | robot stop ; yes | robot release")
    print result
    print "DONE"
elif (message == "START_ROBOT"):
    print "STARTING_APP_MAN"
    print run_as_robot("yes | robot claim ; source ~/.bashrc ; source /opt/ros/diamondback/setup.bash ; source ~/ros/setup.bash ; nohup robot start > ~/robot_start.txt &")
    time.sleep(10.0) #FIXME: this is a race condition. Should wait for the master.
    print run_as_robot("source ~/.bashrc ; source /opt/ros/diamondback/setup.bash ; source ~/ros/setup.bash ; nohup ~/run.sh > ~/run.txt &")
    print "DONE"
else:
    print "REJECT_COMMAND"
    print "action = STOP_ROBOT,START_ROBOT,GET_STATE"
    
