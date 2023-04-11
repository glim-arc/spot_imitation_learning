import matplotlib.pyplot as plt
import numpy as np
import os

def plot():
    model_path = "./"
    avg_loss_list = np.load(os.path.join(model_path,'avg_loss_list.npy'))
    
    #remove outlier
    for i, loss in enumerate(avg_loss_list):
        if loss > avg_loss_list[0]*1.5:
            avg_loss_list[i] = avg_loss_list[0]

    plt.figure()
    epoch = np.arange(1, len(avg_loss_list) + 1)
    plt.plot(epoch, avg_loss_list)
    plt.ylabel('Average Loss')
    plt.xlabel('Epoch')
    plt.title('Average Loss for ' + str(len(avg_loss_list)) + " epoch")
    plt.savefig(os.path.join(model_path,'avg_loss_list.jpg'), dpi=200)
    plt.show()

plot()