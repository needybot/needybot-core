from enum import Enum

class Messages(Enum):
    emergency = '/needybot/msg/emergency'

    instruct = '/needybot/msg/instruct'
    response = '/needybot/msg/response'

    menu_select = '/needybot/msg/menu_select'

    replay = '/needybot/msg/replay'

    reset_timer = '/needybot/msg/reset_timer'
    cancel_timer = '/needybot/msg/pause_timer'

    ui_connected = '/needybot/msg/ui_connected'
    ui_disconnected = '/needybot/msg/ui_disconnected'


class Safety(Enum):
    ground = 'needybot/safety/ground'
    obstacle = 'needybot/safety/obstacle'
    picked_up = 'needybot/safety/picked_up'
    put_down = 'needybot/safety/put_down'


class Status(Enum):
    altitude = 'needybot/status/altitude'
    ipad_battery = 'needybot/status/ipad_battery'
    current_floor = 'needybot/status/current_floor'
    reset_floor = 'needybot/status/reset_floor'


class Task(Enum):
    abort = 'needybot/task/abort'
    did_abort = 'needybot/task/did_abort'
    complete = 'needybot/task/complete'
    fail = 'needybot/task/fail'
