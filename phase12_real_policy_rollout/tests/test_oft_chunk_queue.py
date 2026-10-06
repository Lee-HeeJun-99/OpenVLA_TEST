import sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'02_safety'))
from oft_chunk_queue import OFTChunkQueue,ChunkSafetyError
CHUNK=[[0,0,0,0,0,0,.1*i] for i in range(5)]
class Tests(unittest.TestCase):
 def test_timing_order_partial_duplicate_pending_stale_underrun(self):
  q=OFTChunkQueue();q.enqueue(CHUNK,'a',1);self.assertEqual(range(5),range(len([q.pop(1+.2*i) for i in range(5)])))
  with self.assertRaises(ChunkSafetyError):q.pop(2)
  for chunk,cid in ((CHUNK[:4],'b'),(CHUNK,'a')):
   with self.assertRaises(ChunkSafetyError):q.enqueue(chunk,cid,2)
  q=OFTChunkQueue();q.enqueue(CHUNK,'x',1)
  with self.assertRaises(ChunkSafetyError):q.enqueue(CHUNK,'y',1.1)
  q=OFTChunkQueue(max_age_sec=.5);q.enqueue(CHUNK,'x',1)
  with self.assertRaises(ChunkSafetyError):q.pop(2)
 def test_replace_policy_explicit(self):
  q=OFTChunkQueue(replacement_policy='replace_pending');q.enqueue(CHUNK,'x',1);q.enqueue(CHUNK,'y',1.1)
  self.assertEqual('y',q.pop(1.1).chunk_id)
if __name__=='__main__':unittest.main()
