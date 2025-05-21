# Model performance and validation report
**Source model:** conv1dlstm-medium-balanced-1.h5  
**Generated:** 2025-04-18 15:31:54

### Memory usage
| Model | Model memory | Scratch memory |
| :--- | :--- | :--- |
| float | 68,512 | 16,384 |

### Latency
| Layer | Cycles |
| :--- | :--- |
| CONV_2D | 212,703 |
| CONV_2D | 537,177 |
| CONV_2D | 537,177 |
| MAX_POOL_2D | 35,525 |
| CONV_2D | 537,012 |
| CONV_2D | 936,365 |
| MAX_POOL_2D | 32,191 |
| MEAN | 69,881 |
| FULLY_CONNECTED | 2,189 |
| SOFTMAX | 0 |
| **TOTAL** | **2,900,220** |

### Validation
**Validation data source:** None
