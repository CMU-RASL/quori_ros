import copy

class Emotion():
    def __init__(self, name, emotionDict):
        #emotion should be dict of muscle names: percent
        #at 100% each of the muscles are pulled to a certain percent
        #musclesPercent should be given as values when percentExpressed is 100%
        self.emotionDict = copy.deepcopy(emotionDict)
        self.originalEmotionDict = emotionDict
        self.name = name
    #using the percentExpressed value (Joy 30%) get the respective percentages of each of the muscles
    def getMusclePercent(self, muscle):
        return self.emotionDict[muscle]
    #when an emotion is showing less than 100% modify the percent values for it
    def modifyEmotionValuesByPercent(self, percent):
        # print('percent', percent)
        # print(self.name, percent)
        printed = False
        for muscle in self.emotionDict.keys():
             currPercent = self.originalEmotionDict[muscle]
             newPercent = percent*currPercent
             if not printed:
                # print(muscle, currPercent, newPercent)
                printed = True
             self.emotionDict[muscle] = newPercent
        # print(self.emotionDict)