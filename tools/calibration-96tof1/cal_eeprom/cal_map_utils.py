
def flatten_cal_map(cal):
    lst = []
    for key, list_params in cal.items():
        size, nested_dict = list_params
        lst.append(key)
        lst.append(size)
        for nested_key, nested_value in nested_dict.items():
            param_size, param_value = nested_value
            lst.append(param_size)
            for i in range(int(param_size/4)):
                lst.append(param_value[i])
    return lst


def compare_map(cal1, cal2):
    lst1 = flatten_cal_map(cal1)
    lst2 = flatten_cal_map(cal2)
    ret = True
    for i in range(len(lst1)):
        if(lst1[i]-lst2[i] > 0.2):
            #print(lst1[1], lst2[2])
            ret = False
    return ret
