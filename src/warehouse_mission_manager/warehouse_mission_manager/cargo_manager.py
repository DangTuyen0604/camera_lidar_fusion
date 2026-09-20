class CargoManager:
    """Small explicit cargo invariant used by the mission state machine."""

    def __init__(self):
        self.cargo_id = None

    @property
    def loaded(self):
        return self.cargo_id is not None

    def load(self, cargo_id):
        if self.loaded:
            raise RuntimeError('cargo already loaded')
        self.cargo_id = cargo_id

    def unload(self):
        cargo = self.cargo_id
        self.cargo_id = None
        return cargo
