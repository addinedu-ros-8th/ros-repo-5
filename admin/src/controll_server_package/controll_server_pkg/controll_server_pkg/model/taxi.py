class Taxi:
    def __init__(self, taxi_id, max_passengers):
        self.taxi_id = taxi_id
        self.state = "ready"
        self.location = (0.0, 0.0)
        self.start = (0.0, 0.0)
        self.destination = (0.0, 0.0)
        self.passenger_count = 0
        self.battery = 0.0 
        self.client_id = None
        self.max_passengers = max_passengers

    def assign(self, start_x, start_y, dest_x, dest_y, passenger_count, client_id):
        self.start = (start_x, start_y)
        self.destination = (dest_x, dest_y)
        self.passenger_count = passenger_count
        self.client_id = client_id
        self.state = "assigned"

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
            "taxi_id": self.taxi_id,
            "state": self.state,
            "location": self.location,
            "start": self.start,
            "destination": self.destination,
            "passenger_count": self.passenger_count,
            "client_id": self.client_id,
            "battery": self.battery,
            "max_passengers": self.max_passengers
        }
