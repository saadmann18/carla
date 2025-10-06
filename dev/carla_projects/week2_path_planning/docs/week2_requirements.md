# Week 2 Path Planning - Requirements Document

## Project Overview

This project implements and compares various path planning algorithms for autonomous vehicles in the Carla simulator. The focus is on developing robust, efficient, and safe path planning solutions for different driving scenarios.

## Learning Objectives

By the end of Week 2, students should be able to:

1. **Understand Path Planning Fundamentals**
   - Graph-based search algorithms (A*)
   - Sampling-based algorithms (RRT, RRT*)
   - Trade-offs between optimality and computational efficiency

2. **Implement Core Algorithms**
   - Extract road networks from Carla maps
   - Implement A* pathfinding with proper heuristics
   - Develop RRT and RRT* planners with obstacle avoidance
   - Create PID controllers for path following

3. **Handle Dynamic Environments**
   - Plan paths in the presence of moving obstacles
   - Implement replanning strategies
   - Balance safety and efficiency in traffic scenarios

4. **Evaluate and Compare Algorithms**
   - Define meaningful performance metrics
   - Conduct systematic comparisons
   - Analyze strengths and weaknesses of different approaches

## Technical Requirements

### Day 8: Road Network Extraction
**File**: `road_network_extractor.py`

**Requirements**:
- Extract waypoints from Carla map at configurable intervals (default: 2m)
- Build directed graph representation of road network
- Store waypoint metadata (road_id, lane_id, lane_width)
- Implement graph connectivity based on lane following rules
- Provide visualization capabilities for debugging
- Save/load road network data for reuse

**Deliverables**:
- Functional road network extractor class
- Generated road graph files (waypoints.pkl, road_graph.pkl)
- Visualization of extracted network in Carla

### Day 9: A* Path Planning
**File**: `astar_path_planner.py`

**Requirements**:
- Implement A* algorithm with Euclidean distance heuristic
- Support for weighted graphs (edge costs based on distance)
- Path reconstruction from goal to start
- Calculate path metrics (length, smoothness, direction changes)
- Visualization of planned paths
- Integration with extracted road networks

**Performance Targets**:
- Planning time: < 1 second for typical road networks
- Memory usage: < 100MB for large maps
- Path optimality: Within 5% of optimal solution

**Deliverables**:
- A* planner implementation
- Path visualization plots
- Performance metrics analysis

### Day 10-11: PID Controller Implementation
**File**: `pid_controller.py`

**Requirements**:
- Generic PID controller with configurable gains
- Vehicle-specific controller for steering and speed
- Anti-windup protection for integral term
- Derivative filtering to reduce noise
- Comprehensive logging of control signals
- Path following with lookahead distance

**Control Specifications**:
- Steering control: Cross-track error < 0.5m RMS
- Speed control: Speed error < 2 km/h RMS
- Stability: No sustained oscillations
- Response time: 95% of setpoint within 3 seconds

**Deliverables**:
- PID controller classes with safety features
- Vehicle path following demonstration
- Control performance analysis

### Day 12: RRT Path Planning
**File**: `rrt_planner.py`

**Requirements**:
- Basic RRT implementation with configurable step size
- RRT* variant with path optimization
- Obstacle avoidance with circular obstacles
- Goal-biased sampling (10% probability)
- Tree visualization capabilities
- Performance comparison with A*

**Algorithm Parameters**:
- Step size: 3-5 meters (configurable)
- Max iterations: 5000
- Goal tolerance: 3 meters
- Rewire radius (RRT*): 15 meters

**Deliverables**:
- RRT and RRT* implementations
- Tree growth visualizations
- Path quality comparisons

### Day 13: Dynamic Environment Planning
**File**: `traffic_test_rrt.py`

**Requirements**:
- Dynamic obstacle representation
- Replanning strategies for moving obstacles
- Integration with Carla traffic simulation
- Safety margin enforcement around vehicles
- Performance evaluation in traffic scenarios

**Safety Requirements**:
- Minimum 3m safety distance from vehicles
- Collision-free path guarantee
- Emergency replanning within 2 seconds
- Graceful degradation when no path exists

**Deliverables**:
- Dynamic RRT implementation
- Traffic scenario testing
- Safety analysis report

### Day 14: Algorithm Comparison
**File**: `compare_planners.py`

**Requirements**:
- Systematic comparison framework
- Multiple test scenarios (open, dense, narrow passages)
- Statistical analysis of results (multiple runs)
- Performance metrics calculation
- Comprehensive visualization of results

**Comparison Metrics**:
- Success rate (%)
- Planning time (seconds)
- Path length (meters)
- Path smoothness (direction changes)
- Memory usage (MB)
- Computational complexity

**Test Scenarios**:
1. Open space (no obstacles)
2. Sparse obstacles (3-5 obstacles)
3. Dense obstacles (10+ obstacles)
4. Narrow passages (constrained paths)
5. Maze-like environments

**Deliverables**:
- Comparison framework
- Statistical analysis results
- Recommendation guidelines

## System Requirements

### Software Dependencies
- Python 3.8+
- Carla Simulator 0.9.13+
- Required Python packages:
  - numpy >= 1.19.0
  - matplotlib >= 3.3.0
  - networkx >= 2.5
  - pyyaml >= 5.4.0
  - pandas >= 1.2.0
  - seaborn >= 0.11.0
  - pickle (built-in)

### Hardware Requirements
- CPU: Intel i5 or AMD Ryzen 5 (minimum)
- RAM: 8GB (16GB recommended)
- GPU: GTX 1060 or equivalent (for Carla rendering)
- Storage: 5GB free space

### Carla Setup
- Carla server running on localhost:2000
- Maps: Town01, Town02, Town03 (minimum)
- Traffic simulation capability
- Debug drawing enabled

## Code Quality Standards

### Documentation
- Comprehensive docstrings for all classes and methods
- Inline comments for complex algorithms
- README files for each major component
- Usage examples and tutorials

### Testing
- Unit tests for core algorithms
- Integration tests with Carla
- Performance benchmarks
- Edge case handling

### Code Style
- PEP 8 compliance
- Type hints where appropriate
- Meaningful variable and function names
- Modular design with clear interfaces

## Evaluation Criteria

### Functionality (40%)
- Correct implementation of algorithms
- Integration with Carla simulator
- Handling of edge cases and errors
- Code reliability and robustness

### Performance (30%)
- Meeting specified performance targets
- Efficient memory and CPU usage
- Scalability to larger problems
- Real-time capability demonstration

### Code Quality (20%)
- Clean, readable, and maintainable code
- Proper documentation and comments
- Adherence to coding standards
- Effective use of object-oriented design

### Analysis and Insights (10%)
- Quality of performance analysis
- Meaningful comparisons and conclusions
- Understanding of algorithm trade-offs
- Recommendations for practical use

## Deliverables Summary

1. **Source Code**: Complete implementation of all required files
2. **Documentation**: Requirements, design, and user documentation
3. **Test Results**: Comprehensive testing and validation results
4. **Performance Analysis**: Detailed comparison of algorithms
5. **Demonstration**: Working system demonstration in Carla
6. **Report**: Summary of findings and recommendations

## Timeline

- **Day 8**: Road network extraction and graph building
- **Day 9**: A* algorithm implementation and testing
- **Day 10-11**: PID controller development and tuning
- **Day 12**: RRT algorithm implementation
- **Day 13**: Dynamic environment and traffic testing
- **Day 14**: Comprehensive algorithm comparison and analysis

## Success Criteria

The project is considered successful when:

1. All algorithms are correctly implemented and functional
2. Integration with Carla simulator works reliably
3. Performance targets are met or exceeded
4. Comprehensive comparison analysis is completed
5. Code quality standards are maintained
6. Documentation is complete and accurate
7. System demonstrates practical applicability

## Risk Mitigation

### Technical Risks
- **Carla connectivity issues**: Implement robust connection handling
- **Performance bottlenecks**: Profile code and optimize critical paths
- **Algorithm convergence**: Implement timeout and fallback strategies

### Schedule Risks
- **Complex implementation**: Start with simplified versions and iterate
- **Integration challenges**: Test components independently first
- **Debugging time**: Allocate sufficient time for testing and validation

### Quality Risks
- **Incomplete testing**: Implement automated test suites
- **Poor documentation**: Write documentation alongside code
- **Code maintainability**: Regular code reviews and refactoring
