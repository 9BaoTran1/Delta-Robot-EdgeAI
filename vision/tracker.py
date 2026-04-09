import numpy as np

class KalmanFilter1D:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2, initial_value=0):
        self.q = process_variance
        self.r = measurement_variance
        self.x = initial_value
        self.p = 1.0

    def update(self, z):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (z - self.x)
        self.p = (1 - self.k) * self.p
        return self.x

class StableTracker:
    def __init__(self, max_distance=80, max_age=30):
        self.tracks = {}
        self.next_id = 0
        self.max_distance = max_distance
        self.max_age = max_age

    def update(self, detections):
        results = []
        matched_det = set()
        matched_track = set()
        
        for track_id, (kf_x, kf_y, age, last_seen, cid_hist) in list(self.tracks.items()):
            best_det_idx = None
            best_dist = self.max_distance
            
            for det_idx, (box, score, cid) in enumerate(detections):
                if det_idx in matched_det:
                    continue
                x1, y1, x2, y2 = box
                det_cx = (x1 + x2) / 2.0
                det_cy = (y1 + y2) / 2.0
                dist = np.sqrt((det_cx - kf_x.x) ** 2 + (det_cy - kf_y.x) ** 2)
                
                if dist < best_dist:
                    best_dist = dist
                    best_det_idx = det_idx
            
            if best_det_idx is not None:
                box, score, cid = detections[best_det_idx]
                x1, y1, x2, y2 = box
                det_cx = (x1 + x2) / 2.0
                det_cy = (y1 + y2) / 2.0
                cx_filt = kf_x.update(det_cx)
                cy_filt = kf_y.update(det_cy)
                
                cid_hist.append(cid)
                if len(cid_hist) > 5:
                    cid_hist.pop(0)
                cid_final = max(set(cid_hist), key=cid_hist.count)
                
                results.append((int(cx_filt), int(cy_filt), track_id, cid_final, score))
                matched_det.add(best_det_idx)
                matched_track.add(track_id)
                self.tracks[track_id] = (kf_x, kf_y, age + 1, 0, cid_hist)
        
        for det_idx, (box, score, cid) in enumerate(detections):
            if det_idx in matched_det:
                continue
            x1, y1, x2, y2 = box
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            new_id = self.next_id
            self.next_id += 1
            kf_x = KalmanFilter1D(initial_value=cx)
            kf_y = KalmanFilter1D(initial_value=cy)
            results.append((int(cx), int(cy), new_id, cid, score))
            self.tracks[new_id] = (kf_x, kf_y, 0, 0, [cid])
        
        lost_tracks = []
        for track_id, (kf_x, kf_y, age, lost, cid_hist) in list(self.tracks.items()):
            if track_id not in matched_track:
                lost += 1
                if lost > self.max_age:
                    lost_tracks.append(track_id)
                else:
                    self.tracks[track_id] = (kf_x, kf_y, age, lost, cid_hist)
        
        for track_id in lost_tracks:
            del self.tracks[track_id]
        
        return results
