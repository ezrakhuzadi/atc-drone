use dashmap::DashMap;
use std::hash::Hash;
use std::time::{Duration, Instant};

pub trait CacheEntry {
    fn fetched_at(&self) -> Instant;
}

pub fn prune_cache<K, V>(cache: &DashMap<K, V>, max_entries: usize, max_age: Duration)
where
    K: Clone + Eq + Hash,
    V: CacheEntry,
{
    let now = Instant::now();
    let mut entries: Vec<(K, Instant)> = cache
        .iter()
        .map(|entry| (entry.key().clone(), entry.value().fetched_at()))
        .collect();

    for (key, fetched_at) in &entries {
        if now.duration_since(*fetched_at) > max_age {
            cache.remove(key);
        }
    }

    if cache.len() <= max_entries {
        return;
    }

    entries.sort_by_key(|(_, fetched_at)| *fetched_at);
    for (key, _) in entries {
        if cache.len() <= max_entries {
            break;
        }
        cache.remove(&key);
    }
}
