import airsim 

def qnorm(quaternion):
    while(quaternion.get_length()!=1):
        quaternion = quaternion.sgn()
        iteration = iteration + 1
    return quaternion
    
def quat2vec(quaternion): 
    vector = airsim.Vector3r(quaternion.x_val, quaternion.y_val, quaternion.z_val)
    return vector 
    
def vec2quat(vector): 
    quaternion = airsim.Quaternionr(vector.x_val, vector.y_val, vector.z_val, 0)
    return quaternion
