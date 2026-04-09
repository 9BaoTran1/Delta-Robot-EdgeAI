import cv2
import numpy as np
import os

try:
    import pycuda.driver as cuda
    import tensorrt as trt
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class YOLODetector:
    def __init__(self, engine_path="models/8n_best_ver4.engine"):
        self.engine = None
        self.context = None
        self.inputs = None
        self.outputs = None
        self.bindings = None
        self.stream = None
        self.cuda_ctx = None
        self.engine_ready = False
        self.CLASSES = ['metal', 'paper', 'plastic']
        self.CONF_THRESH = 0.5
        self.IOU_THRESH = 0.40
        self.MAX_BOXES = 10
        self.INPUT_SHAPE = (1, 3, 480, 480)
        
        if not YOLO_AVAILABLE:
            print("CUDA/TensorRT not available")
            return
        
        try:
            cuda.init()
            device = cuda.Device(0)
            self.cuda_ctx = device.make_context()
            TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
            
            if os.path.exists(engine_path):
                with open(engine_path, 'rb') as f:
                    with trt.Runtime(TRT_LOGGER) as runtime:
                        self.engine = runtime.deserialize_cuda_engine(f.read())
            
            if self.engine:
                self.context = self.engine.create_execution_context()
                self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()
                print(f"YOLO Engine Loaded from {engine_path}")
                self.engine_ready = True
            
            self.cuda_ctx.pop()
        except Exception as e:
            print("YOLO Init Error:", e)
            if self.cuda_ctx:
                try: self.cuda_ctx.pop()
                except: pass

    def allocate_buffers(self):
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host = cuda.pagelocked_empty(size, dtype)
            dev = cuda.mem_alloc(host.nbytes)
            bindings.append(int(dev))
            if self.engine.binding_is_input(binding):
                inputs.append({'host': host, 'device': dev})
            else:
                outputs.append({'host': host, 'device': dev})
        return inputs, outputs, bindings, stream
    
    def preprocess(self, img):
        img_resized = cv2.resize(img, (480, 480))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_norm = img_rgb.astype(np.float32) / 255.0
        return np.expand_dims(np.transpose(img_norm, (2, 0, 1)), axis=0)
    
    def infer(self, input_data):
        self.cuda_ctx.push()
        try:
            np.copyto(self.inputs[0]['host'], input_data.ravel())
            cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
            self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
            for out in self.outputs:
                cuda.memcpy_dtoh_async(out['host'], out['device'], self.stream)
            self.stream.synchronize()
            return [out['host'] for out in self.outputs]
        except:
            return None
        finally:
            self.cuda_ctx.pop()

    def nms_class_agnostic(self, boxes, scores, iou_thresh):
        if len(boxes) == 0: return []
        boxes = np.array(boxes)
        scores = np.array(scores)
        x1, y1 = boxes[:, 0], boxes[:, 1]
        x2, y2 = boxes[:, 0] + boxes[:, 2], boxes[:, 1] + boxes[:, 3]
        idxs = np.argsort(scores)[::-1]
        pick = []
        while len(idxs) > 0:
            i = idxs[0]
            pick.append(i)
            if len(idxs) == 1: break
            xx1 = np.maximum(x1[i], x1[idxs[1:]])
            yy1 = np.maximum(y1[i], y1[idxs[1:]])
            xx2 = np.minimum(x2[i], x2[idxs[1:]])
            yy2 = np.minimum(y2[i], y2[idxs[1:]])
            w = np.maximum(0, xx2 - xx1)
            h = np.maximum(0, yy2 - yy1)
            intersection = w * h
            union = (boxes[i][2]*boxes[i][3]) + (boxes[idxs[1:]][:,2]*boxes[idxs[1:]][:,3]) - intersection
            iou = intersection / (union + 1e-6)
            idxs = idxs[1:][iou <= iou_thresh]
        return pick
    
    def postprocess(self, output, orig_shape):
        num_classes = len(self.CLASSES)
        if len(output.shape) == 1:
            output = output.reshape(4 + num_classes, output.size // (4 + num_classes)).T
        elif output.shape[0] == 1:
            output = output[0].T
        
        H, W = orig_shape
        boxes, scores, class_ids = [], [], []
        
        for det in output:
            class_scores = det[4:4+num_classes]
            score = float(np.max(class_scores))
            if score < self.CONF_THRESH: continue
            class_id = int(np.argmax(class_scores))
            cx, cy, w, h = det[:4]
            x1 = int((cx - w/2) * W / 480)
            y1 = int((cy - h/2) * H / 480)
            w_s = int(w * W / 480)
            h_s = int(h * H / 480)
            boxes.append([max(0, x1), max(0, y1), w_s, h_s])
            scores.append(score)
            class_ids.append(class_id)
        
        if not boxes: return []
        indices = self.nms_class_agnostic(boxes, scores, self.IOU_THRESH)
        return [([boxes[i][0], boxes[i][1], boxes[i][0]+boxes[i][2], boxes[i][1]+boxes[i][3]], scores[i], class_ids[i]) for i in indices[:self.MAX_BOXES]]
    
    def detect(self, frame):
        if not self.engine_ready: return []
        inp = self.preprocess(frame)
        out = self.infer(inp)
        if out is None: return []
        return self.postprocess(out[0], frame.shape[:2])
