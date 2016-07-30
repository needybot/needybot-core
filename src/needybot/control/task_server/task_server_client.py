from std_srvs.srv import Empty

from needybot.services.base import BaseServiceClient
from needybot.lib.patterns import Singleton

from needybot_srvs.srv import NextTask 

class TaskServerClient(BaseServiceClient):

    def __init__(self, ns=''):
        namespace = 'task_server' if len(ns) == 0 else '{}_task_server'.format(ns)
        super(TaskServerClient, self).__init__(namespace, [
            ('abort', NextTask),
            ('boot', Empty),
            ('complete', NextTask),
            ('fail_step', Empty),
            ('shutdown', Empty),
            ('succeed_step', Empty),
        ])

