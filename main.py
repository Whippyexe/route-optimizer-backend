from fastapi import FastAPI, Request
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = FastAPI()

@app.post("/optimize_route/")
async def optimize_route(request: Request):
    data = await request.json()
    locations = data["locations"]  # list of (lat, lon)

    # Distance callback
    def create_distance_callback(locations):
        from geopy.distance import geodesic
        def distance(from_index, to_index):
            return int(geodesic(locations[from_index], locations[to_index]).meters)
        return distance

    manager = pywrapcp.RoutingIndexManager(len(locations), 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    distance_callback = create_distance_callback(locations)
    transit_callback_index = routing.RegisterTransitCallback(lambda i, j: distance_callback(manager.IndexToNode(i), manager.IndexToNode(j)))
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_params)

    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return {"optimized_route": route}
    else:
        return {"error": "No solution found"}
