# multiAgents.py
# --------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        #chosenIndex = random.choice(bestIndices) # Pick randomly among the best
        bestMoves = []
        for i in range(len(legalMoves)):
            if i in bestIndices:
                bestMoves.append(legalMoves[i])
        chosenMoves = random.choice(bestMoves) # Pick randomly among the best
        return chosenMoves

        "Add more of your code here if you want to"

        #return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        for game, state in zip(newGhostStates, newScaredTimes):
            if newPos == game.getPosition() and state == 0:
                return -1000
        
        foodCount = len(newFood.asList())
        
        if foodCount == 0:
            closest = 0
        else:
            closest = 100
        for food in newFood.asList():
            if manhattanDistance(newPos, food) < closest:
                closest = manhattanDistance(newPos, food)
        
        return -foodCount + 0.5 / (closest + 1)



        #return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()

        return self.maxValue(gameState, self.depth, 0)[1]

    def value(self, gameState, depth, agent):
        if gameState.isLose() or gameState.isWin() or depth == 0:
            return self.evaluationFunction(gameState)
        if agent == 0:
            return self.maxValue(gameState, depth, agent)[0]
        if agent > 0:
            return self.minValue(gameState, depth, agent)

    def maxValue(self, gameState, depth, agent):
        val = -float('inf')
        best = None
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            nextValue = self.value(nextState, depth, 1)
            if nextValue > val:
                val = nextValue
                best = action
        return (val, best)

    def minValue(self, gameState, depth, agent):
        val = float('inf')
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            if agent == gameState.getNumAgents() - 1:
                val = min(val, self.value(nextState, depth - 1, 0))
            else:
                val = min(val, self.value(nextState, depth, agent + 1))
        return val



class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()
        return self.maxValue(gameState, self.depth, 0, -float('inf'), float('inf'))[1]



    def value(self, gameState, depth, agent, alpha, beta):
        if gameState.isLose() or gameState.isWin() or depth == 0:
            return self.evaluationFunction(gameState)
        if agent == 0:
            return self.maxValue(gameState, depth, agent, alpha, beta)[0]
        if agent > 0:
            return self.minValue(gameState, depth, agent, alpha, beta)

    def maxValue(self, gameState, depth, agent, alpha, beta):
        val = -float('inf')
        best = None
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            nextValue = self.value(nextState, depth, 1, alpha, beta)
            if nextValue > val:
                val = nextValue
                best = action
            if val > beta:
                return (val, best)
            alpha = max(val, alpha)
        return (val, best)

    def minValue(self, gameState, depth, agent, alpha, beta):
        val = float('inf')
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            if agent == gameState.getNumAgents() - 1:
                val = min(val, self.value(nextState, depth - 1, 0, alpha, beta))
            else:
                val = min(val, self.value(nextState, depth, agent + 1, alpha, beta))
            if val < alpha:
                return val
            beta = min(val, beta)
        return val



class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        #util.raiseNotDefined()
        return self.maxValue(gameState, self.depth, 0)[1]

    def value(self, gameState, depth, agent):
        if gameState.isLose() or gameState.isWin() or depth == 0:
            return self.evaluationFunction(gameState)
        if agent == 0:
            return self.maxValue(gameState, depth, agent)[0]
        if agent > 0:
            return self.expectValue(gameState, depth, agent)

    def maxValue(self, gameState, depth, agent):
        val = -float('inf')
        best = None
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            nextValue = self.value(nextState, depth, 1)
            if nextValue > val:
                val = nextValue
                best = action
        return (val, best)

    def expectValue(self, gameState, depth, agent):
        val = 0
        chance = 0
        for action in gameState.getLegalActions(agent):
            nextState = gameState.generateSuccessor(agent, action)
            if agent == gameState.getNumAgents() - 1:
                val += self.value(nextState, depth - 1, 0)
            else:
                val += self.value(nextState, depth, agent + 1)
            chance += 1
        return val/chance

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    position = currentGameState.getPacmanPosition()
    food = currentGameState.getFood()
    ghostStates = currentGameState.getGhostStates()
    scaredTimes = [ghostState.scaredTimer for ghostState in ghostStates]
    for ghost, sTime in zip(ghostStates, scaredTimes):
        if position == ghost.getPosition() and sTime == 0:
            return -1000
    foodCount = len(food.asList())
    dists = [0] if foodCount == 0 else []
    for food in food.asList():
        dists.append(manhattanDistance(position, food))
    return - foodCount + .5/(min(dists) + 1) + currentGameState.getScore()/100

# Abbreviation
better = betterEvaluationFunction
