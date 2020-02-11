###
# Two joints Leg Forward kinematic
#
# Inputs:
#   l1: link1 length
#   l2: link2 length
#   (a, b): end effect coordinate
#
# Output:
#   T: Homogeneous Transformation Matrix of End Effect
#
# Formulas:
#   a = c01 * x12 - s01 * y12 + x01
#   b = s01 * x12 + c01 * y12 + y01
#   l1 ** 2 = x01 ** 2 + y01 ** 2
#   l2 ** 2 = x12 ** 2 + y12 ** 2
#   theta01 = arctan(y01 / x01)
#   theta12 = arctan(y12 / x12)
#   theta02 = theta01 + theta12
###

