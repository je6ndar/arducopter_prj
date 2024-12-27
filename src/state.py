import signal
import threading

# IDLE, CAPTURE_FRAME, HAS_DIR, CAPTURE_TARGET, INIT_FRAME, TRACK_TARGET
# "01_IDLE", "02_..." to support > , < etc.

# states
IDLE = "01_IDLE"
HOVER = "02_HOVER"

ANY = "00_ANY"

# events

EV_RC_LOW = "EV_RC_LOW"
#EV_RC_MED = "EV_RC_MED"
EV_RC_HIGH = "EV_RC_HIGH"

EV_MKDIR = "EV_MKDIR"
EV_FRAME = "EV_FRAME"

# CurrentState,Event,NextState
# S0,Event,S
StateTransitions = [
    [IDLE, EV_RC_HIGH, HOVER],
    [HOVER, EV_RC_LOW, IDLE]
]

# StateTransitionsDict = {S0: {event: S, ..}, ..}
StateTransitionsDict = {}
for S0,event,S in StateTransitions:
    if S0 not in StateTransitionsDict:
        StateTransitionsDict[S0] = {}
    StateTransitionsDict[S0][event] = S

#print(StateTransitions)
#print(StateTransitionsDict)

# other combinations - do nothing next_state == cur_state

# init state
STATE = IDLE

_state_lock = threading.Lock()

# state transition
def next_state(event):
    # TODO - add lock
    global STATE

    with _state_lock:
        new_state = StateTransitionsDict.get(STATE, {}).get(event)
        if not new_state:
            new_state = StateTransitionsDict.get(ANY, {}).get(event)

        if not new_state:
            print("State transition not allowed, state is the same: %s" % STATE)
            print('STATE:%s -> EVENT:%s -> ?' % (STATE, event))
        else:
            print('STATE:%s -> EVENT:%s -> %s' % (STATE, event, new_state))
            STATE = new_state

    return STATE


# signals - to test state transitions without mavlink
# signal.SIGHUP - RC_LOW
# signal.SIGUSR1, signal.SIGUSR2 - RC_MED, RC_HIGH

signal_event_map = {
    signal.SIGHUP: EV_RC_LOW,
    #signal.SIGUSR1: EV_RC_MED,
    signal.SIGUSR2: EV_RC_HIGH
}

def state_signal_handler(signum, frame):
    #print("in state_signal_handler", signum, frame)
    s = signal.Signals(signum)
    event = signal_event_map.get(s)
    if event:
        next_state(event)



signal.signal(signal.SIGHUP, state_signal_handler) #  aka RC_LOW
#signal.signal(signal.SIGUSR1, state_signal_handler) #  aka RC_MED
signal.signal(signal.SIGUSR2, state_signal_handler) #  aka RC_HIGH
