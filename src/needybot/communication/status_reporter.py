#!/usr/bin/env python

import re

import rospy
from socket_client import NeedyClient


class StatusReporterClient(NeedyClient):

    """
    Child of NeedyClient class found in src/core/communication/socket_client.

    Overrides `opened` method to construct and send a payload of all Needybot
    parameters in ropsy (namespace => '/needybot'), then shut itself down.

    Minor overrides to other instance methods `closed` and `shutdown` are added
    for messaging purposes.
    """
    
    def __init__(self, url):
        super(NeedyClient, self).__init__(url)

    def opened(self):

        """
        Handler for a successful opened socket connection.

        Overrides parent method with a custom payload that is displatched
        to the 'needy:status' channel upon a successful connection to
        the socket server.

        Payload sent is a general status report on Needybot's current
        situation derived by all known rospy params in the '/needybot' ns.
        """

        rospy.loginfo('[Socket]: connection opened successfully')
        if not self.join():
            rospy.loginfo('[Socket]: already joined')

        # get all parameters starting with '/needybot' and crunch
        # them into a dictionary to be sent as a status update.    
        status = {'_'.join(parameter.split('/')[-2:]): rospy.get_param(parameter) for
                  parameter in rospy.get_param_names() if
                  re.match('/needybot', parameter)}

        try:
            self.push(status)
            self.shutdown('Needybot status sent successfully to socket server')
        except Exception as e:
            self.shutdown(e)
        
    def shutdown(self, message):

        """
        Override of parent method that simply logs a passed message before
        calling the WebSocketClient `close` method.

        Args:
            messasge (str): the message to report to the log file.
        """

        rospy.loginfo(message)
        self.close()

    def report(self):

        """
        Convenience method used for semantic purposes.

        Calls own connect method.
        """

        self.connect()
        
def report_status(timer_event):

    """
    Routine method called by rospy.Timer class in the main thread.

    Creates a StatusReporterClient and calls its report method to snatch up
    all of the Needybot parameters and send them to the Websocket server.
    """
    
    status_reporter = StatusReporterClient('ws://nb-services.herokuapp.com/ws')
    status_reporter.report()
    
if __name__ == '__main__':
    rospy.init_node('status_reporter')
    rospy.Timer(rospy.Duration(rospy.get_param('/needybot/status_report_timer')), report_status)

    while not rospy.is_shutdown():
        rospy.spin()
