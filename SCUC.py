from pyomo.environ import *

# Define the Abstract Model
model = AbstractModel()

#Sets
model.G=Set(initialize=['G1', 'G2', 'G3'])
model.T = Set(initialize=[1, 2, 3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24], ordered=True)
model.demand=Set(initialize=[1,2,3])
model.B=Set(initialize=[1,2,3,4])
#model.L=Set(within=model.B*model.B)
model.L=Set(initialize=[1,2,3,4])
#model.loadlocation=Set(dimen=2)
#model.genlocation=Set(dimen=2)

#Parameters
model.x=Param(model.L)
model.D=Param(model.B,model.T)
model.Pmax=Param(model.G)
model.Pmin=Param(model.G)
model.a=Param(model.G)
model.b=Param(model.G)
model.SD=Param(model.G)
model.SU=Param(model.G)
model.Tup=Param(model.G)
model.Tdn=Param(model.G)
model.Lmax=Param(model.L)
model.From = Param(model.L)         # Starting bus of each line
model.To = Param(model.L)           # Ending bus of each line
model.M = Param(initialize=1000) # Big-M value - choose a large value


#Variables
model.P=Var(model.G, model.T, within=NonNegativeReals)
model.I=Var(model.G, model.T, within=Binary)
model.Z=Var(model.G, model.T, within=Binary)
model.Y=Var(model.G, model.T, within=Binary)
model.Xup=Var(model.G, model.T, within=Integers)
model.Xdn=Var(model.G, model.T, within=Integers)
model.PL=Var(model.L, model.T, within=Reals)
model.theta=Var(model.B,model.T, within=Reals)
model.A = Var(model.G, model.T, within=Binary) # New binary variable for linearization


#Objective function
def objective_rule(model):
    return sum(model.a[i]*model.P[i,t]+model.b[i]+model.SU[i]*model.Y[i,t]+model.SD[i]*model.Z[i,t] for i in model.G for t in model.T)
model.CostFunction=Objective(rule=objective_rule, sense=minimize)

#Generation unit Constraints
def GenMax_rule(model,i,t):
    return (model.P[i,t] <= model.Pmax[i]*model.I[i,t])
model.GenMaxLimit=Constraint(model.G,model.T, rule=GenMax_rule)

def GenMin_rule(model,i,t):
    return (model.P[i,t] >= model.Pmin[i]*model.I[i,t])
model.GenMinLimit=Constraint(model.G,model.T, rule=GenMin_rule)

def RamUp_rule(model,i,t):
    if t>1:
       return ((model.P[i,t]-model.P[i,t-1]) <= model.Pmin[i]*model.Y[i,t])
    else:
       return Constraint.Skip
model.RamUpLimit=Constraint(model.G,model.T, rule=RamUp_rule)

def RamdDn_rule(model,i,t):
    if t>1:
       return ((model.P[i,t-1]-model.P[i,t]) <= model.Pmin[i]*model.Z[i,t])
    else:
       return Constraint.Skip
model.RamDnLimit=Constraint(model.G,model.T, rule=RamDn_rule)

def StartUpLogic_rule(model,i,t):
    if t == 1:
        return model.Y[i, t] >= model.I[i, t]
    else:
        return model.Y[i, t] >= model.I[g, t] - model.I[g, t - 1]

model.StartupLogic = Constraint(model.G, model.T, rule=StartUpLogic_rule)

def ShutDownLogic_rule(model, i, t):
    if t == 1:
       return model.Z[i,t] >= 1 - model.I[i,t]
    else:
       return model.Z[i,t] >= model.I[i,t - 1] - model.I[i, t]

model.ShutdownLogic = Constraint(model.G, model.T, rule=ShutDownLogic_rule)

def TurnOn_rule(model,i,t):
    if t>1:
        return (model.Xup[i,t-1]-model.Tup[i])  <= model.M * (1 - model.A[i,t])
    else:
        return Constraint.Skip
model.TurnOnLimit=Constraint(model.G,model.T, rule=TurnOn_rule)

def TurnOff_rule(model,i,t):
    if t>1:
        return (model.Xdn[i,t-1]-model.Tdn[i])  <= model.M * (model.A[i,t])
    else:
        return Constraint.Skip
model.TurnOffLimit=Constraint(model.G,model.T, rule=TurnOff_rule)

#Power Balance Limit
def power_balance_rule(model, n, t):
    gen = sum(model.P[i, t] for i in model.G)  # Sum of generation injected at bus n
    load = sum (model.D[n, t])  # Load at bus n
    inflow = sum(1 / model.x[l] * (model.theta[model.From[l], t] - model.theta[model.To[l], t]) for l in model.L if model.To[l] == n)
    outflow = sum(1 / model.x[l] * (model.theta[model.From[l], t] - model.theta[model.To[l], t]) for l in model.L if model.From[l] == n)
    return gen - load + inflow - outflow == 0

model.PowerBalance = Constraint(model.B, model.T, rule=power_balance_rule)

# Power Flow Limits Constraints
def power_flow_limit_rule(model, l, t):
    flow = (model.theta[model.From[l], t] - model.theta[model.To[l], t]) / model.x[l]
    return -model.Lmax[l] <= flow <= model.Lmax[l]



# Constraints to relate A to the product of (I[i,t-1]-I[i,t])
def A_rule1(model, i, t):
    if t>1:
        return model.A[i,t] <= model.I[i,t-1] 
    else:
        return Constraint.Skip
model.ALimit1 = Constraint(model.G, model.T, rule=A_rule1)

def A_rule2(model, i, t):
    if t>1:
        return model.A[i,t] <= 1 - model.I[i,t]
    else:
        return Constraint.Skip
model.ALimit2 = Constraint(model.G, model.T, rule=A_rule2)

def A_rule3(model, i, t):
    if t>1:
        return model.A[i,t] >= model.I[i,t-1] - model.I[i,t]
    else:
        return Constraint.Skip
model.ALimit3 = Constraint(model.G, model.T, rule=A_rule3)

#Create Instance
data=DataPortal()
data.load(filename="SCUC.dat", model=model)
instance=model.create_instance(data)
instance.pprint()

#Solving model
optimizer=SolverFactory("glpk")
optimizer.solve(instance)
instance.display()