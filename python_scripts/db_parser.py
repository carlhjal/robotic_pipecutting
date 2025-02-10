from reach import load
import numpy as np

def load_db(path):

    # Load database from ROS message
    db = load(path)
    result = db.results[-1]

    positions = np.array([r.goal()[0:3, 3] for r in result])
    scores = np.array([r.score for r in result])
    return positions

data = load_db("reach.db.xml")

print(data)