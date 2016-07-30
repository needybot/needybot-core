import rospy


class BaseServiceClient(object):
    """
    Class for interacting directly with the DialogServer (the service).

    Other nodes in the Needybot ecosphere will interact with the service
    via this class.
    """

    def __init__(self, namespace, services):
        self.namespace = namespace
        map(self.service_builder, services)

    def service_builder(self, service_def):

        """
        Method for generating service endpoints for this Client.

        Each service contains a type and a service definition, as
        defined in the service's .srv file.

        For each service, a method is attached to the client that provides an
        endpoint with which other nodes can interact with the service.

        Because services must be uniquely namespaced, each service proxy will be
        exposed externally omitting the class's namespace as defined at
        construction time, but mapping to a service that uses the service
        class's namepspace. This means service proxies map as follows

        self.method('some message')       ->       /self.namespace_method

        Args:
            service (string): the name of the service
        """

        service_name, service_type = service_def
        namespaced_service = '{}_{}'.format(self.namespace, service_name)

        def service_method(self, *args, **kwargs):
            rospy.wait_for_service(namespaced_service)
            service_proxy = rospy.ServiceProxy(namespaced_service, service_type)
            return service_proxy(self, *args, **kwargs)

        setattr(self, service_name, service_method)


class BaseService(object):

    """
    Class intended to be used as a base class for future service classes.

    Provides interface requirement for a rate at instantiation, as well
    as a service registration method for convenience.
    """

    def __init__(self, namespace, rate=None):
        self.namespace = namespace
        self.rate = rate
        self.registered = []

    def register_services(self, services):

        """
        Similar to `pub_sub.create_publishers` and `pub_sub.create_subscribers`,
        this method takes a list of service definition pairs, and registers
        a `rospy.Service` object for each.

        Because services must be uniquely namespaced, each service will be
        exposed externally using the class's namespace as defined at
        construction time. This means services map as follows

        /self.namespace_method       ->       self.method(req)

        Each pair consists of the name of the service and its service type, as
        defined by a .srv file.
        """

        for service in services:
            callback_name, service_type = service
            service_name = '{}_{}'.format(self.namespace, callback_name)
            callback = getattr(self, callback_name)
            self.registered.append(rospy.Service(service_name, service_type, callback))

    def shutdown(self):
        for service in self.registered:
            service.shutdown('Shutting down base service.')

    def start(self, req):
        raise NotImplementedError('Must be overridden in a child class.')

    def stop(self, req):
        raise NotImplementedError('Must be overridden in a child class.')
