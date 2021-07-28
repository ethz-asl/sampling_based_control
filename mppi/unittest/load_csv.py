import csv
import pandas as pd
import numpy as np


def to_numpy(x):
    return np.asarray(eval(x))


converters = {
    "int": eval,
    "vector_eigen_vectors": to_numpy,
    "eigen_vector": to_numpy
}
df = pd.read_csv('test-2021-07-28-13-20.csv', sep=';', converters=converters)

# print(df.columns.values)
# print(df["vector_eigen_vectors"])
i = df["double"][0]
print(i)
print(type(i))

v = df["vector_eigen_vectors"][0]
print(v)

v2 = df["eigen_vector"][0]
print(v2)
