"""Integration log metrics; command ACK never implies task success."""
import math
from collections import Counter

def collect(rows, events):
    positions=[r['observation']['tcp_m_abc'][:3] for r in rows if 'observation' in r]
    times=[r['observation']['receive_monotonic'] for r in rows if 'observation' in r]
    close=[r.get('target_time') for r in rows if r.get('canonical_action',[0]*7)[6]>=.7]
    ack=[e['ack_latency'] for e in events if e.get('ack_latency') is not None]
    return dict(task_success=None,completion_time_s=max(times)-min(times) if times else None,
        trajectory_length_m=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),
        tcp_deviation_m=None,first_close_intent_time_s=min(close) if close else None,
        safety_rejection_count=sum(bool(r.get('safety_blockers')) for r in rows),
        holds=sum(e.get('command_type')=='hold' for e in events),ack_latency_s=ack,
        blockers=dict(Counter(b for r in rows for b in r.get('safety_blockers',[]))),
        oft_k_index_reject=dict(Counter(r['chunk_index'] for r in rows if r.get('safety_blockers'))))
