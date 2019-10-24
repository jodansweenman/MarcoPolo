import boto3
import rospy

polly_client = boto3.Session().client('polly')

# Example point ... this will come from ros later
ball_xy_marcoframe = (-1.5, 100.9)

# Determine text for lateral position
if ball_xy_marcoframe[0] > 0:
	left_right_text = 'to my right'
else:
	left_right_text = 'to my left'

# Determine text for longitudinal position
if ball_xy_marcoframe[1] > 0:
	forward_back_text = 'ahead of me'
else:
	forward_back_text = 'behind me'

full_ball_text = 'The ball is located %s meters %s and %s meters %s.' % (abs(ball_xy_marcoframe[0]),
							   left_right_text,
							   abs(ball_xy_marcoframe[1]),
							   forward_back_text)

print(full_ball_text)

response = polly_client.synthesize_speech(VoiceId='Joanna',
                OutputFormat='mp3', 
                Text = full_ball_text)

file = open('speech.mp3', 'wb')
file.write(response['AudioStream'].read())
file.close()
