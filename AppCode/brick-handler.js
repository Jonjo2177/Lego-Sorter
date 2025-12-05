import { createClient } from '@blinkdotnew/sdk';

const blink = createClient({
  projectId: 'lego-sorter-companion-app-45ib1hz3',
  authRequired: false
});

// When a brick is sorted
async function onBrickSorted(color, count = 1) {
  const timestamp = new Date().toISOString();
  const id = `brick_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  
  // Insert to database
  await blink.db.sortedBricks.create({
    id,
    color: color.toLowerCase(), // 'red', 'blue', 'yellow', etc.
    count,
    timestamp
  });
  
  // Publish real-time update (dashboard updates instantly)
  const channel = blink.realtime.channel('lego-sorter');
  await channel.publish('brick_sorted', {
    color,
    count,
    timestamp
  });
}