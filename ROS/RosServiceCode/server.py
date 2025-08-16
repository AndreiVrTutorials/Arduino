from my_service_pkg.srv import AddTwoInts, AddTwoIntsResponse
import rospy
def handle_add_two_ints(req):
    print(f"Adding {req.a} + {req.b}")
    return AddTwoIntsResponse(req.a + req.b)
def server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Server ready.")
    rospy.spin()
if __name__ == "__main__":
    server()