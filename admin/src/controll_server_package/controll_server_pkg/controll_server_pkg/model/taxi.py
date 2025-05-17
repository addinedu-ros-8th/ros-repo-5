# self.state = ready dispatch drive_start boarded drive_destination landing completed charged

class Taxi:
    def __init__(self, vehicle_id, max_passengers):
        self.vehicle_id = vehicle_id
        self.state = "ready"
        self.location = (0.0, 0.0)
        self.start = (0.0, 0.0)
        self.start_node = None
        self.destination = (0.0, 0.0)
        self.destination_node = None
        self.passenger_count = 0
        self.battery = 0.0 
        self.passenger_id = None
        self.max_passengers = max_passengers
        self.dispatches_id = None
        self.passenger_state = None

    def dispatch(self, start_x, start_y, start_node, dest_x, dest_y, destination_node, passenger_count, passenger_id, dispatches_id):
        self.start = (start_x, start_y)
        self.start_node = start_node
        self.destination = (dest_x, dest_y)
        self.destination_node = destination_node
        self.passenger_count = passenger_count
        self.passenger_id = passenger_id
        self.dispatches_id = dispatches_id
        self.state = "dispatch"

    def update_location(self, x, y):
        self.location = (x, y)

    def update_battery(self, percent):
        self.battery = percent

    def update_state(self, state: str):
        self.state = state

    def is_available(self, min_battery: float = 60.0, required_passengers: int = 1):
        return (
            self.state == "ready"
            and self.battery >= min_battery
            and self.max_passengers >= required_passengers
            and self.location is not None
        )

    def to_dict(self):
        return {
            "vehicle_id": self.vehicle_id,
            "state": self.state,
            "location": self.location,
            "start": self.start,
            "start_node": self.start_node,
            "destination": self.destination,
            "destination_node": self.destination_node,
            "passenger_count": self.passenger_count,
            "battery": self.battery,
            "max_passengers": self.max_passengers,
            "passenger_id": self.passenger_id,
            "dispatches_id": self.dispatches_id,
            "passenger_state": self.passenger_state
        }
