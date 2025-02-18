from leaderboard.autoagents.autonomous_agent import AutonomousAgent

def get_entry_point():
    return 'PDMCarlaPlanner'

class PDMCarlaPlanner(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self._agent = PDMCarlaPlannerAgent(path_to_conf_file)

    def tick(self, input_data):
        return self._agent.run_step(input_data)

    def finish(self):
        self._agent.finish()