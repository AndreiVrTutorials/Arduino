import rospy
import sys
from my_service_pkg.srv import AddTwoInts

def client(x, y):
    rospy.init_node('add_two_ints_client')
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp = add_two_ints(x, y)
        print(f"Result: {resp.sum}")
    except rospy.ServiceException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Please pass two numbers as arguments.")
        sys.exit(1)
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    client(x, y)