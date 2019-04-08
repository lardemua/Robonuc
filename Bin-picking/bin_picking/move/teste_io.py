import sys
import copy
import rospy

from robonuc.msg import ios
# from generate_plan_move import monitoring_ios

# definir o publicador
io_pub = rospy.Publisher(
                        '/io_client_messages',
                        ios,
                        queue_size = 10)


print "============ Starting test_io"

rospy.init_node('bin_picking_test_io',
                anonymous=True)
rate = rospy.Rate(10) # 10hz


# function - 1 to read, 2 to switch on and 3 to switch of the respective IO number (ionumber)
def monitoring_ios(function,ionumber):
    cod = function*10 + ionumber
    io_msg = ios()
    io_msg.code = cod
    print "Setting I/Os code:"
    print cod
    io_pub.publish(io_msg)

while raw_input('') != 'c':

    print "Press 'y' to open gripper!!!"
    if raw_input('') == 'y':
        # IO number 8 activates the suction
        monitoring_ios(2,4)
        monitoring_ios(2,8)

    print "Press 'y' to close gripper!!!"
    if raw_input('') == 'y':               
        # IO number 8 activates the suction
        monitoring_ios(3,8)
        monitoring_ios(3,4)

    print "Press 'c' to end!!!"